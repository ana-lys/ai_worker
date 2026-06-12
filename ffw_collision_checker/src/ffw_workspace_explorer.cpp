#include "ffw_ik_solver.h"
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <fstream>
#include <chrono>
#include <cmath>
#include <random>
#include <atomic>
#include <deque>
#include <unordered_map>
#include <tuple>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

const double VOXEL_SIZE = 0.05;

struct VoxelKey {
    int x, y, z;
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash {
    std::size_t operator()(const VoxelKey& k) const {
        std::size_t h1 = std::hash<int>{}(k.x);
        std::size_t h2 = std::hash<int>{}(k.y);
        std::size_t h3 = std::hash<int>{}(k.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

// Configuration
const double X_MIN = 0.4;
const double X_MAX = 0.8;
const double Y_MIN = -0.65;
const double Y_MAX = 0.0;
const double Z_MIN = 0.6;
const double Z_MAX = 2.0;
int GRID_SIZE = 3;
int NUM_THREADS = 12;
const double DELTA = M_PI / 8.0;
const int NUM_DELTA_SAMPLES = 20; // Generate 20 perturbations per IK target
const int MAX_IK_STEPS = 200;

struct TargetPose {
    Eigen::Isometry3d pose;
};

struct ResultPose {
    double x, y, z;
    double qx, qy, qz, qw;
};

std::mutex result_mtx;
std::vector<ResultPose> pareto_front;
std::atomic<int> num_ik_solved{0};
std::atomic<int> num_delta_packs_solved{0};
std::atomic<int> jobs_completed{0};
std::atomic<int> target_queue_idx{0};

void worker_thread(const std::string& xml_path, const std::vector<TargetPose>& targets, Eigen::Isometry3d init_l) {
    char error[1000];
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "Worker thread MuJoCo load error: " << error << "\n";
        return;
    }
    mjData* d = mj_makeData(m);
    int right_id = mj_name2id(m, mjOBJ_SITE, "right_gripper_site");

    ffw_ik::SolverConfig solver_cfg;
    solver_cfg.damping = 2e-3;
    solver_cfg.step_size = 0.15;
    solver_cfg.tolerance = 5e-3;
    solver_cfg.track_orientation = true;
    solver_cfg.ee_improvement_rate = 1e-5;
    solver_cfg.ee_window = 15;
    solver_cfg.ori_weight = 0.5;
    solver_cfg.left_weight_scale = 1.0 / 24.0;
    solver_cfg.right_weight_scale = 1.0;

    ffw_ik::CollisionCostConfig col_cfg;
    col_cfg.collision_margin = 0.10;
    col_cfg.weight_scale = 0.005;

    ffw_ik::IKSolver solver(m);
    
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> dist(-DELTA, DELTA);

    std::vector<ResultPose> local_results;
    local_results.reserve(5000);

    while (true) {
        int i = target_queue_idx.fetch_add(1);
        if (i >= targets.size()) break;

        // Reset robot to neutral
        mju_zero(d->qpos, m->nq);
        mju_zero(d->qvel, m->nv);
        if (m->nq >= 7 && m->jnt_type[0] == mjJNT_FREE)
            d->qpos[3] = 1.0;
        mj_forward(m, d);

        Eigen::Isometry3d target_r = targets[i].pose;
        std::deque<double> err_hist, dist_hist;

        // Run IK
        for (int step = 0; step < MAX_IK_STEPS; ++step) {
            ffw_ik::StepResult res = solver.solveStep(d, init_l, target_r, solver_cfg, col_cfg, err_hist, dist_hist);
            if (res.stalled) break;
        }
        
        num_ik_solved++;

        // Delta perturbation
        std::vector<double> base_qpos(m->nq);
        for (int j = 0; j < m->nq; ++j) base_qpos[j] = d->qpos[j];

        // Sample perturbations
        for (int p = 0; p < NUM_DELTA_SAMPLES; ++p) {
            // Apply delta and clamp to joint limits
            for (int j = 0; j < m->nq; ++j) {
                if (j >= 7 || m->jnt_type[0] != mjJNT_FREE) {
                    double val = base_qpos[j] + dist(rng);
                    int jid = m->dof_jntid[j]; // Assuming 1 DOF per joint for simple hinges/sliders
                    if (m->jnt_limited[jid]) {
                        double q_min = m->jnt_range[2 * jid];
                        double q_max = m->jnt_range[2 * jid + 1];
                        val = std::max(q_min, std::min(q_max, val));
                    }
                    d->qpos[j] = val;
                }
            }
            
            // Forward kinematics and Collision detection
            mj_kinematics(m, d);
            mj_comPos(m, d);
            mj_collision(m, d);

            // Check if there is any collision (distance < 0 or some small margin)
            bool collision = false;
            for (int c = 0; c < d->ncon; ++c) {
                if (d->contact[c].dist < 0.02) { // 2cm margin for safety
                    collision = true;
                    break;
                }
            }

            if (!collision && right_id >= 0) {
                Eigen::Vector3d pos = Eigen::Vector3d::Map(d->site_xpos + 3 * right_id);
                Eigen::Matrix3d rot = Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(d->site_xmat + 9 * right_id).cast<double>();
                Eigen::Quaterniond q(rot);
                
                local_results.push_back({pos.x(), pos.y(), pos.z(), q.x(), q.y(), q.z(), q.w()});
            }
            num_delta_packs_solved++;
        }

        jobs_completed++;

        // Periodically flush local results to global to save memory/time
        if (local_results.size() > 1000) {
            std::lock_guard<std::mutex> lock(result_mtx);
            pareto_front.insert(pareto_front.end(), local_results.begin(), local_results.end());
            local_results.clear();
        }
    }

    // Flush remaining
    if (!local_results.empty()) {
        std::lock_guard<std::mutex> lock(result_mtx);
        pareto_front.insert(pareto_front.end(), local_results.begin(), local_results.end());
    }

    mj_deleteData(d);
    mj_deleteModel(m);
}

int main(int argc, char** argv) {
    int MODE = 2; // 0=explore only, 1=post-process only, 2=both
    if (argc > 1) {
        MODE = std::stoi(argv[1]);
    }
    if (argc > 2) {
        GRID_SIZE = std::stoi(argv[2]);
    }
    if (argc > 3) {
        NUM_THREADS = std::stoi(argv[3]);
    }
    
    std::string xml_path = ament_index_cpp::get_package_share_directory("ffw_collision_checker") + "/3rd_party/robotis_ffw/scene_inverse_kinematic.xml";
    std::vector<double> ori_vals = {-M_PI/2.0, -M_PI/4.0, 0.0, M_PI/4.0, M_PI/2.0};

    if (MODE == 0 || MODE == 2) {
        std::cout << "Starting 12D Workspace Explorer with GRID_SIZE=" << GRID_SIZE << ", NUM_THREADS=" << NUM_THREADS << std::endl;
        // Generate Targets
        std::vector<TargetPose> targets;
        
        double dx = (GRID_SIZE > 1) ? (X_MAX - X_MIN) / (GRID_SIZE - 1) : 0.0;
        double dy = (GRID_SIZE > 1) ? (Y_MAX - Y_MIN) / (GRID_SIZE - 1) : 0.0;
        double dz = (GRID_SIZE > 1) ? (Z_MAX - Z_MIN) / (GRID_SIZE - 1) : 0.0;

    for (int ix = 0; ix < GRID_SIZE; ++ix) {
        for (int iy = 0; iy < GRID_SIZE; ++iy) {
            for (int iz = 0; iz < GRID_SIZE; ++iz) {
                // Hollow check: only keep outer faces
                if (ix != 0 && ix != GRID_SIZE - 1 &&
                    iy != 0 && iy != GRID_SIZE - 1 &&
                    iz != 0 && iz != GRID_SIZE - 1) {
                    continue; 
                }

                double x = X_MIN + ix * dx;
                double y = Y_MIN + iy * dy;
                double z = Z_MIN + iz * dz;

                for (double r : ori_vals) {
                    for (double p : ori_vals) {
                        for (double yw : ori_vals) {
                            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                            pose.translation() = Eigen::Vector3d(x, y, z);
                            
                            Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
                            Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
                            Eigen::AngleAxisd yawAngle(yw, Eigen::Vector3d::UnitZ());
                            Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
                            pose.linear() = q.matrix();
                            
                            targets.push_back({pose});
                        }
                    }
                }
            }
        }
    }

    std::cout << "Generated " << targets.size() << " targets on the hollow bounding box boundary." << std::endl;

    // We need an initial left arm pose so it doesn't flail
    char error[1000];
    mjModel* temp_m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!temp_m) {
        std::cerr << "Main thread MuJoCo load error: " << error << "\n";
        return 1;
    }
    mjData* temp_d = mj_makeData(temp_m);
    mju_zero(temp_d->qpos, temp_m->nq);
    mju_zero(temp_d->qvel, temp_m->nv);
    if (temp_m->nq >= 7 && temp_m->jnt_type[0] == mjJNT_FREE) temp_d->qpos[3] = 1.0;
    mj_forward(temp_m, temp_d);
    
    int left_id = mj_name2id(temp_m, mjOBJ_SITE, "left_gripper_site");
    Eigen::Isometry3d init_l = Eigen::Isometry3d::Identity();
    if (left_id >= 0) {
        init_l.translation() = Eigen::Vector3d::Map(temp_d->site_xpos + 3 * left_id);
        init_l.linear() = Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(temp_d->site_xmat + 9 * left_id).cast<double>();
    }
    mj_deleteData(temp_d);
    mj_deleteModel(temp_m);

    // Launch Threads
    std::vector<std::thread> threads;
    for (int i = 0; i < NUM_THREADS; ++i) {
        threads.emplace_back(worker_thread, xml_path, std::ref(targets), init_l);
    }

    // Monitoring thread
    bool running = true;
    std::thread monitor([&]() {
        while (running) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            int completed = jobs_completed.load();
            int ik_solved = num_ik_solved.load();
            int delta_packs = num_delta_packs_solved.load();
            
            std::cout << "\r[Progress] Targets: " << completed << "/" << targets.size() 
                      << " | IK Solves: " << ik_solved 
                      << " | Delta FK Poses: " << delta_packs << std::flush;
            
            // Periodic save
            std::lock_guard<std::mutex> lock(result_mtx);
            std::ofstream out("explore/pareto_front_right.csv");
            if (!out.is_open()) out.open("pareto_front_right.csv"); // Fallback
            out << "x,y,z,qx,qy,qz,qw\n";
            for (const auto& p : pareto_front) {
                out << p.x << "," << p.y << "," << p.z << "," 
                    << p.qx << "," << p.qy << "," << p.qz << "," << p.qw << "\n";
            }
            out.close();
        }
    });

    for (auto& t : threads) {
        t.join();
    }
    
    running = false;
    monitor.join();
    
    std::cout << "\nExploration complete! Total Pareto points: " << pareto_front.size() << std::endl;
    } // End of MODE == 0 || MODE == 2

    if (MODE == 1 || MODE == 2) {
        if (MODE == 1) {
            std::cout << "Loading CSV for post-processing..." << std::endl;
            std::ifstream in("src/ai_worker/ffw_collision_checker/explore/pareto_front_right.csv");
            if (!in.is_open()) {
                in.open("explore/pareto_front_right.csv");
            }
            if (!in.is_open()) {
                in.open("pareto_front_right.csv");
            }
            if (!in.is_open()) {
                std::cerr << "Failed to open pareto_front_right.csv" << std::endl;
                return 1;
            }
            std::string line;
            std::getline(in, line); // Skip header
            while (std::getline(in, line)) {
                std::stringstream ss(line);
                std::string token;
                ResultPose p;
                if (std::getline(ss, token, ',')) p.x = std::stod(token);
                if (std::getline(ss, token, ',')) p.y = std::stod(token);
                if (std::getline(ss, token, ',')) p.z = std::stod(token);
                if (std::getline(ss, token, ',')) p.qx = std::stod(token);
                if (std::getline(ss, token, ',')) p.qy = std::stod(token);
                if (std::getline(ss, token, ',')) p.qz = std::stod(token);
                if (std::getline(ss, token, ',')) p.qw = std::stod(token);
                if (p.x >= 0.2) {
                    pareto_front.push_back(p);
                }
            }
            std::cout << "Loaded " << pareto_front.size() << " points." << std::endl;
        }

    // --- POST-PROCESSING ---
    std::cout << "Starting C++ Post-Processing (Voxelization & Boundary Extraction)..." << std::endl;
    
    std::unordered_map<VoxelKey, std::vector<Eigen::Quaterniond>, VoxelHash> voxels;
    
    for (const auto& p : pareto_front) {
        int ix = static_cast<int>(std::floor(p.x / VOXEL_SIZE));
        int iy = static_cast<int>(std::floor(p.y / VOXEL_SIZE));
        int iz = static_cast<int>(std::floor(p.z / VOXEL_SIZE));
        voxels[{ix, iy, iz}].push_back(Eigen::Quaterniond(p.qw, p.qx, p.qy, p.qz));
    }
    
    std::cout << "Total populated voxels: " << voxels.size() << std::endl;

    // Target Orientations
    std::vector<Eigen::Quaterniond> target_quats;
    for (double r : ori_vals) {
        for (double p : ori_vals) {
            for (double yw : ori_vals) {
                Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
                Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
                Eigen::AngleAxisd yawAngle(yw, Eigen::Vector3d::UnitZ());
                target_quats.push_back(yawAngle * pitchAngle * rollAngle);
            }
        }
    }

    std::cout << "Calculating Base 125-Scores for ALL voxels..." << std::endl;
    std::unordered_map<VoxelKey, std::vector<double>, VoxelHash> base_scores;
    for (const auto& pair : voxels) {
        std::vector<double> scores(target_quats.size(), -1.0);
        for (size_t i = 0; i < target_quats.size(); ++i) {
            for (const auto& rq : pair.second) {
                double dot = target_quats[i].dot(rq);
                double cos_theta = 2.0 * (dot * dot) - 1.0;
                if (cos_theta > scores[i]) scores[i] = cos_theta;
            }
        }
        base_scores[pair.first] = scores;
    }

    std::cout << "Extracting All Voxel Categories..." << std::endl;
    std::ofstream out_voxels("src/ai_worker/ffw_collision_checker/explore/pareto_boundary_voxels.csv");
    if (!out_voxels.is_open()) out_voxels.open("explore/pareto_boundary_voxels.csv");
    if (!out_voxels.is_open()) out_voxels.open("pareto_boundary_voxels.csv");
    out_voxels << "x,y,z,versatility_score";
    for (size_t i = 0; i < target_quats.size(); ++i) out_voxels << ",score_" << i;
    out_voxels << "\n";

    std::ofstream out_bin("src/ai_worker/ffw_collision_checker/explore/pareto_boundary_voxels.bin", std::ios::binary);
    if (!out_bin.is_open()) out_bin.open("explore/pareto_boundary_voxels.bin", std::ios::binary);
    if (!out_bin.is_open()) out_bin.open("pareto_boundary_voxels.bin", std::ios::binary);

    int directions[6][3] = {
        {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
    };
    int boundary_count = 0;
    int blocked_count = 0;
    int interior_count = 0;

    int min_ix = 1000, max_ix = -1000;
    int min_iy = 1000, max_iy = -1000;
    int min_iz = 1000, max_iz = -1000;
    for (const auto& pair : voxels) {
        min_ix = std::min(min_ix, pair.first.x);
        max_ix = std::max(max_ix, pair.first.x);
        min_iy = std::min(min_iy, pair.first.y);
        max_iy = std::max(max_iy, pair.first.y);
        min_iz = std::min(min_iz, pair.first.z);
        max_iz = std::max(max_iz, pair.first.z);
    }
    
    // Explicitly extend X minimum to 0.0 and maximum to 0.9 to capture blocked regions
    int x_min_cap_idx = static_cast<int>(std::floor(0.0 / VOXEL_SIZE));
    int x_max_cap_idx = static_cast<int>(std::floor(0.9 / VOXEL_SIZE));
    min_ix = std::min(min_ix, x_min_cap_idx);
    max_ix = std::max(max_ix, x_max_cap_idx);
    
    // Add 1 voxel padding to Y and Z to ensure the workspace is fully encased in hard blocks
    min_iy -= 1;
    max_iy += 1;
    min_iz -= 1;
    max_iz += 1;

    for (int ix = min_ix; ix <= max_ix; ++ix) {
        for (int iy = min_iy; iy <= max_iy; ++iy) {
            for (int iz = min_iz; iz <= max_iz; ++iz) {
                VoxelKey k = {ix, iy, iz};
                
                double vx = (k.x + 0.5) * VOXEL_SIZE;
                double vy = (k.y + 0.5) * VOXEL_SIZE;
                double vz = (k.z + 0.5) * VOXEL_SIZE;
                
                if (voxels.find(k) == voxels.end()) {
                    // BLOCKED VOXEL
                    blocked_count++;
                    out_voxels << vx << "," << vy << "," << vz << ",0.0";
                    for (size_t i = 0; i < target_quats.size(); ++i) out_voxels << ",0.0";
                    out_voxels << "\n";
                    
                    std::vector<float> bin_row(129, 0.0f);
                    bin_row[0] = vx; bin_row[1] = vy; bin_row[2] = vz; bin_row[3] = 0.0f;
                    out_bin.write(reinterpret_cast<const char*>(bin_row.data()), bin_row.size() * sizeof(float));
                } else {
                    bool is_boundary = false;
                    for (int dx = -1; dx <= 1 && !is_boundary; ++dx) {
                        for (int dy = -1; dy <= 1 && !is_boundary; ++dy) {
                            for (int dz = -1; dz <= 1 && !is_boundary; ++dz) {
                                if (dx == 0 && dy == 0 && dz == 0) continue;
                                VoxelKey neighbor = {k.x + dx, k.y + dy, k.z + dz};
                                if (voxels.find(neighbor) == voxels.end()) {
                                    is_boundary = true;
                                }
                            }
                        }
                    }

                    if (is_boundary) {
                        // SOFT BLOCK (Pareto Boundary)
                        boundary_count++;
                        std::vector<double> smoothed(target_quats.size(), 0.0);
                        double sum_score = 0.0;
                        for (size_t i = 0; i < target_quats.size(); ++i) {
                            double my_score = base_scores[k][i];
                            double max_neighbor = -1.0;
                            for (int j = 0; j < 6; ++j) {
                                VoxelKey neighbor = {k.x + directions[j][0], k.y + directions[j][1], k.z + directions[j][2]};
                                auto it = base_scores.find(neighbor);
                                if (it != base_scores.end()) {
                                    if (it->second[i] > max_neighbor) {
                                        max_neighbor = it->second[i];
                                    }
                                }
                            }
                            double neighbor_term = max_neighbor;
                            if (max_neighbor > 0.0) {
                                neighbor_term = 0.25 * max_neighbor;
                            }
                            smoothed[i] = std::max(my_score, neighbor_term);
                            sum_score += smoothed[i];
                        }
                        double versatility = sum_score / target_quats.size();
                        out_voxels << vx << "," << vy << "," << vz << "," << versatility;
                        for (double s : smoothed) out_voxels << "," << s;
                        out_voxels << "\n";

                        std::vector<float> bin_row(129, 0.0f);
                        bin_row[0] = vx; bin_row[1] = vy; bin_row[2] = vz; bin_row[3] = versatility;
                        for (size_t i = 0; i < target_quats.size(); ++i) bin_row[4+i] = smoothed[i];
                        out_bin.write(reinterpret_cast<const char*>(bin_row.data()), bin_row.size() * sizeof(float));
                    } else {
                        // NO BLOCK (Interior)
                        interior_count++;
                        out_voxels << vx << "," << vy << "," << vz << ",1.0";
                        for (size_t i = 0; i < target_quats.size(); ++i) out_voxels << ",1.0";
                        out_voxels << "\n";
                        
                        std::vector<float> bin_row(129, 1.0f);
                        bin_row[0] = vx; bin_row[1] = vy; bin_row[2] = vz; bin_row[3] = 1.0f;
                        out_bin.write(reinterpret_cast<const char*>(bin_row.data()), bin_row.size() * sizeof(float));
                    }
                }
            }
        }
    }
    out_voxels.close();
    if (out_bin.is_open()) out_bin.close();

    std::cout << "Total processed voxels in bounding box: " << (blocked_count + boundary_count + interior_count) << std::endl;
    std::cout << "  - Blocked (Unreached/Capped): " << blocked_count << std::endl;
    std::cout << "  - Boundary (Soft Block):      " << boundary_count << std::endl;
    std::cout << "  - Interior (No Block):        " << interior_count << std::endl;
    std::cout << "Done! Saved all voxels to explore/pareto_boundary_voxels.csv" << std::endl;
    } // End of MODE == 1 || MODE == 2

    return 0;
}
