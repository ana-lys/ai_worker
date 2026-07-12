// scan_to_map_icp.hpp
//
// Fast 2D point-to-line ICP for correcting odometry by matching a live LIDAR
// scan (which contains scattered/dynamic clutter) against a static,
// pre-filtered map feature set (e.g. wall lines reduced to points).
//
// Design choices made for performance:
//  1. The map is static -> build the KD-tree AND per-point local-line normals
//     exactly ONCE (in the constructor), never per-frame.
//  2. Point-to-LINE (point-to-plane in 2D) residuals are used instead of
//     point-to-point: for wall-like structure this converges in far fewer
//     iterations and is far less biased by partial overlap.
//  3. Each iteration solves a 3x3 linear system in closed form (Cholesky) --
//     no SVD needed, unlike point-to-point ICP.
//  4. Outlier / dynamic-object rejection: correspondences beyond a distance
//     threshold are dropped, and a Huber weight is applied to the rest, so
//     scattered non-map objects in the scan barely influence the result.
//  5. Nearest-neighbor search uses nanoflann (header-only, extremely fast
//     KD-tree), and the per-point loop is OpenMP-parallel.
//
// Dependencies: Eigen3, nanoflann.hpp (bundled alongside this file).
//
// NOTE ON OpenMP: the #pragma omp directives below only activate if you compile
// with -fopenmp. Benchmarked at LakiBeam1 x2 @20Hz scale (~600-1600 pts/frame,
// map ~600-2500 pts), single-threaded is actually 15-30% FASTER than OpenMP --
// the thread-sync overhead exceeds the per-point work at this size. Recommend
// building WITHOUT -fopenmp for this sensor/point-count class. The pragmas are
// harmless no-ops if you don't pass -fopenmp, and only worth enabling once your
// map or scan regularly exceeds roughly 5k-10k points.

#pragma once

#include "nanoflann.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace icp2d {

// ---------------------------------------------------------------------
// Basic types
// ---------------------------------------------------------------------

struct Point2D {
  double x, y;
};

// 2D rigid transform: rotation theta + translation (tx,ty).
// Applies as: p' = R(theta) * p + t
struct Pose2D {
  double x = 0.0, y = 0.0, theta = 0.0;

  Eigen::Matrix2d R() const {
    const double c = std::cos(theta), s = std::sin(theta);
    Eigen::Matrix2d m;
    m << c, -s, s, c;
    return m;
  }
  Eigen::Vector2d t() const { return Eigen::Vector2d(x, y); }

  Point2D apply(const Point2D &p) const {
    const double c = std::cos(theta), s = std::sin(theta);
    return Point2D{c * p.x - s * p.y + x, s * p.x + c * p.y + y};
  }

  // this * other  (compose transforms, this applied after other)
  Pose2D operator*(const Pose2D &o) const {
    Eigen::Vector2d tt = R() * o.t() + t();
    return Pose2D{tt.x(), tt.y(), theta + o.theta};
  }

  Pose2D inverse() const {
    Eigen::Matrix2d Rt = R().transpose();
    Eigen::Vector2d ti = -Rt * t();
    return Pose2D{ti.x(), ti.y(), -theta};
  }
};

// ---------------------------------------------------------------------
// nanoflann adaptor over a flat vector<Point2D>
// ---------------------------------------------------------------------

struct PointCloudAdaptor {
  const std::vector<Point2D> &pts;
  explicit PointCloudAdaptor(const std::vector<Point2D> &p) : pts(p) {}

  inline size_t kdtree_get_point_count() const { return pts.size(); }
  inline double kdtree_get_pt(size_t idx, size_t dim) const {
    return dim == 0 ? pts[idx].x : pts[idx].y;
  }
  template <class BBOX> bool kdtree_get_bbox(BBOX &) const { return false; }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloudAdaptor>, PointCloudAdaptor,
    2 /*dims*/, size_t>;

// ---------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------

struct ICPConfig {
  int max_iterations = 30;
  double max_correspondence_dist =
      0.5; // meters; correspondences farther than this
           // are rejected outright (handles scan points
           // that hit dynamic/non-map objects).
  double huber_delta =
      0.10; // meters; robust-weight knee for remaining matches.
  double translation_eps = 1e-4; // convergence: |delta t| below this...
  double rotation_eps = 1e-5;    // ...and |delta theta| below this -> stop.
  int normal_k_neighbors = 8;    // # of neighbors used to fit local line/normal
                                 // on the STATIC MAP (computed once).
  double min_inlier_ratio =
      0.25; // if fewer than this fraction of scan points
            // find a valid correspondence, report failure
            // (e.g. scan mostly outside map / too much clutter).
  bool verbose = false;
};

struct ICPResult {
  Pose2D correction; // T that maps the *initial-guess-aligned* scan onto the
                     // map frame
  Pose2D corrected_pose; // full corrected pose in map frame = correction *
                         // initial_guess
  bool converged = false;
  int iterations = 0;
  int inlier_count = 0;
  double inlier_rms = std::numeric_limits<double>::infinity();
};

// ---------------------------------------------------------------------
// ScanToMapICP
// ---------------------------------------------------------------------
//
// Usage:
//   ScanToMapICP matcher(static_map_points);           // build once, reuse
//   every frame ICPResult r = matcher.align(live_scan_points,
//   odom_guess_in_map_frame); Pose2D corrected_odom = r.corrected_pose; // use
//   this as the corrected pose
//
class ScanToMapICP {
public:
  explicit ScanToMapICP(std::vector<Point2D> map_points,
                        const ICPConfig &cfg = ICPConfig())
      : map_points_(std::move(map_points)), adaptor_(map_points_),
        tree_(2, adaptor_,
              nanoflann::KDTreeSingleIndexAdaptorParams(16 /*leaf*/)),
        cfg_(cfg) {
    tree_.buildIndex();
    computeMapNormals();
  }

  // scan_points: live scan in the SENSOR/ROBOT frame.
  // initial_guess: current odometry estimate expressed as a pose in the MAP
  // frame
  //                (i.e. the transform that would take scan_points into the map
  //                frame
  //                 if odometry had no drift). This is your prior from
  //                 wheel/IMU odom.
  ICPResult align(const std::vector<Point2D> &scan_points,
                  const Pose2D &initial_guess) const {
    ICPResult result;
    Pose2D estimate = initial_guess;

    const size_t N = scan_points.size();
    std::vector<Point2D> transformed(N);

    std::vector<int32_t> corr_map_idx(N);
    std::vector<uint8_t> corr_valid(N);
    std::vector<double> corr_dist2(N);

    int iter = 0;
    for (; iter < cfg_.max_iterations; ++iter) {
      // 1) transform scan by current estimate
      for (long i = 0; i < (long)N; ++i)
        transformed[i] = estimate.apply(scan_points[i]);

      // 2) nearest-neighbor correspondence search (parallel), with distance
      // gating
      const double max_d2 =
          cfg_.max_correspondence_dist * cfg_.max_correspondence_dist;
      for (long i = 0; i < (long)N; ++i) {
        size_t nn_idx;
        double nn_dist2;
        nanoflann::KNNResultSet<double, size_t> resultSet(1);
        resultSet.init(&nn_idx, &nn_dist2);
        const double query[2] = {transformed[i].x, transformed[i].y};
        tree_.findNeighbors(resultSet, query, nanoflann::SearchParameters());

        if (nn_dist2 <= max_d2) {
          corr_map_idx[i] = (int32_t)nn_idx;
          corr_dist2[i] = nn_dist2;
          corr_valid[i] = 1;
        } else {
          corr_valid[i] = 0; // rejected: likely a dynamic/non-map object
        }
      }

      // 3) build the linearized point-to-line system:
      //    residual r_i = n_i . (T(dx,dy,dtheta) * p_i - q_i)
      //    minimized over small increment (dx, dy, dtheta) around current
      //    estimate.
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
      Eigen::Vector3d b = Eigen::Vector3d::Zero();
      int inliers = 0;
      double sq_err_sum = 0.0;

      for (size_t i = 0; i < N; ++i) {
        if (!corr_valid[i])
          continue;
        const int32_t mi = corr_map_idx[i];
        if (!map_has_normal_[mi])
          continue; // isolated map point, no reliable line

        const Eigen::Vector2d p(transformed[i].x, transformed[i].y);
        const Eigen::Vector2d q(map_points_[mi].x, map_points_[mi].y);
        const Eigen::Vector2d n = map_normals_[mi];

        const double residual = n.dot(p - q);

        // robust (Huber) weight
        const double a = std::abs(residual);
        const double w = (a <= cfg_.huber_delta) ? 1.0 : (cfg_.huber_delta / a);

        // Jacobian of residual wrt (dx, dy, dtheta), evaluated at the current
        // linearization point p = R*p_local + t:
        //   d(residual)/d(dx,dy)    = n^T
        //   d(residual)/d(dtheta)   = n^T * [ -p_y, p_x ]^T  (rotation
        //   derivative)
        Eigen::Vector3d J;
        J(0) = n.x();
        J(1) = n.y();
        J(2) = n.x() * (-p.y()) + n.y() * (p.x());

        H += w * (J * J.transpose());
        b += -w * residual * J;

        sq_err_sum += residual * residual;
        ++inliers;
      }

      result.inlier_count = inliers;
      if (inliers > 0)
        result.inlier_rms = std::sqrt(sq_err_sum / inliers);

      if (inliers < (int)(cfg_.min_inlier_ratio * std::min(map_points_.size(), N)) || inliers < 5) {
        // too little valid overlap between scan and map this frame
        result.converged = false;
        result.iterations = iter;
        break;
      }

      // 4) solve 3x3 system (small regularization for numerical safety)
      H.diagonal().array() += 1e-9;
      Eigen::LDLT<Eigen::Matrix3d> ldlt(H);
      Eigen::Vector3d delta = ldlt.solve(b);

      // 5) apply increment: estimate = Delta(delta) * estimate
      Pose2D d;
      d.x = delta(0);
      d.y = delta(1);
      d.theta = delta(2);
      estimate = d * estimate;

      if (cfg_.verbose) {
        std::printf("[ICP Debug] Iter %d: inliers=%d, rms=%.4f, delta=[%.6f, %.6f, %.6f], estimate=[%.4f, %.4f, %.4f]\n",
                    iter, inliers, inliers > 0 ? std::sqrt(sq_err_sum / inliers) : 0.0,
                    delta(0), delta(1), delta(2), estimate.x, estimate.y, estimate.theta);
      }

      result.iterations = iter + 1;

      if (delta.head<2>().norm() < cfg_.translation_eps &&
          std::abs(delta(2)) < cfg_.rotation_eps) {
        result.converged = true;
        break;
      }
    }

    result.corrected_pose = estimate;
    result.correction = estimate * initial_guess.inverse();
    return result;
  }

  const std::vector<Point2D> &mapPoints() const { return map_points_; }

private:
  // Precompute, once, a unit normal for each map point by local PCA over its
  // k nearest neighbors within the map itself. This turns "wall points" into
  // "wall points + line direction", enabling point-to-line residuals.
  void computeMapNormals() {
    const size_t M = map_points_.size();
    map_normals_.assign(M, Eigen::Vector2d::Zero());
    map_has_normal_.assign(M, false);

    const size_t K = (size_t)cfg_.normal_k_neighbors;
    for (long i = 0; i < (long)M; ++i) {
      std::vector<size_t> idx(K);
      std::vector<double> dist2(K);
      nanoflann::KNNResultSet<double, size_t> resultSet(K);
      resultSet.init(idx.data(), dist2.data());
      const double query[2] = {map_points_[i].x, map_points_[i].y};
      tree_.findNeighbors(resultSet, query, nanoflann::SearchParameters());
      const size_t found = resultSet.size();
      if (found < 3)
        continue;

      Eigen::Vector2d mean = Eigen::Vector2d::Zero();
      for (size_t k = 0; k < found; ++k)
        mean += Eigen::Vector2d(map_points_[idx[k]].x, map_points_[idx[k]].y);
      mean /= (double)found;

      Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
      for (size_t k = 0; k < found; ++k) {
        Eigen::Vector2d d(map_points_[idx[k]].x - mean.x(),
                          map_points_[idx[k]].y - mean.y());
        cov += d * d.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
      // eigenvector of SMALLEST eigenvalue = normal to the local line direction
      Eigen::Vector2d normal = es.eigenvectors().col(0).normalized();

      map_normals_[i] = normal;
      map_has_normal_[i] = true;
    }
  }

  std::vector<Point2D> map_points_;
  PointCloudAdaptor adaptor_;
  KDTree tree_;
  ICPConfig cfg_;

  std::vector<Eigen::Vector2d> map_normals_;
  std::vector<uint8_t> map_has_normal_;
};

} // namespace icp2d