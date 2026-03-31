#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <proxsuite/proxqp/dense/dense.hpp>

#include <Eigen/Dense>

#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace {

pinocchio::SE3 makeTarget(const Eigen::Vector3d& t, const Eigen::Vector3d& rpy)
{
    // Orientation part is kept for completeness but not used in position-only IK
    const double cr = std::cos(rpy.x()), sr = std::sin(rpy.x());
    const double cp = std::cos(rpy.y()), sp = std::sin(rpy.y());
    const double cy = std::cos(rpy.z()), sy = std::sin(rpy.z());
    Eigen::Matrix3d R;
    R << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
         sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
            -sp,                    cp * sr,                    cp * cr;
    return pinocchio::SE3(R, t);
}

} // namespace

int main()
{
    const std::string urdf_path =
        "/home/analys/robotis_ws/src/ai_worker/ffw_description/urdf/ffw_bg2_rev4_follower/ffw_bg2_follower.urdf";

    try {
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path, model);
        pinocchio::Data data(model);

        const std::string left_frame_name_primary = "arm_l_link7";
        const std::string right_frame_name_primary = "arm_r_link7";

        pinocchio::FrameIndex lf = model.getFrameId(left_frame_name_primary);
        pinocchio::FrameIndex rf = model.getFrameId(right_frame_name_primary);

        if (lf == model.nframes || rf == model.nframes) {
            throw std::runtime_error("End-effector frame not found in model");
        }

        const pinocchio::SE3 target_l = makeTarget(Eigen::Vector3d(0.25, 0.20, 0.50), Eigen::Vector3d(0.3, 0.0, 0.1));
        const pinocchio::SE3 target_r = makeTarget(Eigen::Vector3d(0.5, -0.20, 1.20), Eigen::Vector3d(0.0, 0.2, -0.1));

        Eigen::VectorXd q = pinocchio::neutral(model);

        // Print initial end‑effector poses at the neutral configuration.
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::framesForwardKinematics(model, data, q);
        const pinocchio::SE3 init_l = data.oMf[lf];
        const pinocchio::SE3 init_r = data.oMf[rf];
        std::cout << "Initial EE poses (neutral q):\n";
        std::cout << "  Left  T: " << init_l.translation().transpose() << "\n";
        std::cout << "  Right T: " << init_r.translation().transpose() << "\n";

        const double damping = 1e-3;
        const double step = 0.05;
        const double tol = 1e-4;               // position tolerance (m)
        const int max_iter = 100;
        const double joint_vel_limit = 0.6;    // rad/s

        for (int iter = 0; iter < max_iter; ++iter) {
            pinocchio::forwardKinematics(model, data, q);
            pinocchio::framesForwardKinematics(model, data, q);
            pinocchio::computeJointJacobians(model, data, q);

            const pinocchio::SE3 current_l = data.oMf[lf];
            const pinocchio::SE3 current_r = data.oMf[rf];

            // Position error only (world frame) – we ignore orientation
            Eigen::VectorXd err(6);
            err.head<3>() = current_l.translation() - target_l.translation(); // left
            err.tail<3>() = current_r.translation() - target_r.translation(); // right

            if (err.norm() < tol) {
                std::cout << "Converged in " << iter << " iterations\n";
                break;
            }
            if (iter % 20 == 0) {
                std::cout << "Iter " << iter << ", err norm: " << err.norm()
                          << ", left T: " << current_l.translation().transpose()
                          << ", right T: " << current_r.translation().transpose() << "\n";
            }

            // Compute full Jacobians (6xN) and extract linear parts (3xN)
            Eigen::MatrixXd Jl(6, model.nv);
            Eigen::MatrixXd Jr(6, model.nv);
            Jl.setZero();
            Jr.setZero();
            pinocchio::computeFrameJacobian(model, data, q, lf, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jl);
            pinocchio::computeFrameJacobian(model, data, q, rf, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jr);

            // Use only linear velocity rows (first 3)
            Eigen::MatrixXd J(6, model.nv);
            J.topRows<3>() = Jl.topRows<3>();
            J.bottomRows<3>() = Jr.topRows<3>();

            // Build inequality constraints: joint velocity limits
            const int nv = model.nv;
            std::vector<Eigen::RowVectorXd> C_rows;
            std::vector<double> l_rows;
            std::vector<double> u_rows;

            for (int j = 0; j < nv; ++j) {
                Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(nv);
                row[j] = 1.0;
                C_rows.push_back(row);
                l_rows.push_back(-joint_vel_limit);
                u_rows.push_back(joint_vel_limit);
            }

            const int m = static_cast<int>(C_rows.size());
            Eigen::MatrixXd C(m, nv);
            Eigen::VectorXd l(m), u(m);
            for (int i = 0; i < m; ++i) {
                C.row(i) = C_rows[i];
                l[i] = l_rows[i];
                u[i] = u_rows[i];
            }

            // QP: minimize 0.5||J v + err||^2 + 0.5*damping*||v||^2
            Eigen::MatrixXd H = damping * Eigen::MatrixXd::Identity(nv, nv) + J.transpose() * J;
            // Enforce exact symmetry (required by ProxQP)
            H = 0.5 * (H + H.transpose());
            Eigen::VectorXd g = J.transpose() * err;

            // Solve QP
            proxsuite::proxqp::dense::QP<double> qp(nv, 0, m);
            qp.settings.verbose = false;
            qp.settings.eps_abs = 1e-6;
            qp.settings.eps_rel = 0.0;
            qp.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;

            qp.init(H, g,
                    std::nullopt, std::nullopt,   // no equalities
                    C, l, u,                       // inequalities
                    std::nullopt, std::nullopt,    // no variable bounds
                    false, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
            qp.solve();

            if (qp.results.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
                std::cerr << "QP did not solve (status " << static_cast<int>(qp.results.info.status)
                          << "); stopping at iter " << iter << "\n";
                break;
            }

            const Eigen::VectorXd v = qp.results.x;
            q = pinocchio::integrate(model, q, step * v);
        }

        std::cout << "IK loop finished.\n";

        pinocchio::framesForwardKinematics(model, data, q);
        const pinocchio::SE3 final_l = data.oMf[lf];
        const pinocchio::SE3 final_r = data.oMf[rf];

        std::cout << "Final joint configuration (" << q.size() << " values):\n";
        std::cout << q.transpose() << "\n\n";
        std::cout << "Left EE pose translation: " << final_l.translation().transpose() << "\n";
        std::cout << "Right EE pose translation: " << final_r.translation().transpose() << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}