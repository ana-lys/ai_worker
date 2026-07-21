#pragma once

#include "rf2o_laser_odometry/scan_to_map_icp.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <vector>
namespace icp2d {

struct LineSegment {
  double phi; // Normal angle in radians
  double rho; // Perpendicular distance to origin
  double length;
  Point2D p1; // Endpoint 1
  Point2D p2; // Endpoint 2
};

struct LineMatch {
  double map_phi;
  double map_rho;
  double scan_rho;
};

class GeometricRelocalizer {
public:
  explicit GeometricRelocalizer(const std::vector<Point2D> &map_points)
      : map_points_(map_points) {
    map_segs_ = extractSegmentsFromMap();
    std::printf("[Relocalizer] Extracted %zu line segments from the map.\n",
                map_segs_.size());
  }

  std::optional<Pose2D> relocalize(const std::vector<Point2D> &scan_points,
                                    const ScanToMapICP &matcher,
                                    const std::optional<Pose2D> &initial_guess = std::nullopt) const {
    std::vector<LineSegment> scan_segs = extractSegmentsFromScan(scan_points);

    std::vector<double> theta_hypotheses = searchRotation(scan_segs, map_segs_);
    std::vector<Pose2D> candidates =
        solveTranslation(scan_segs, map_segs_, theta_hypotheses);

    if (initial_guess.has_value()) {
      candidates.push_back(*initial_guess);
    }

    Pose2D best_pose{0.0, 0.0, 0.0};
    double best_score = -1.0;

    // No geometric candidates AND no initial guess — truly no information
    if (candidates.empty()) {
      return std::nullopt;
    }

    for (const auto &cand : candidates) {
      ICPResult res = matcher.align(scan_points, cand);
      double score = res.inlier_count;
      if (res.converged && score > best_score) {
        best_score = score;
        best_pose = res.corrected_pose;
      }
    }

    if (best_score < 0.0) {
      // None of the candidates converged in ICP — return the initial guess
      // as a degraded prior (it was pushed into candidates so we know it exists).
      return initial_guess;
    }

    return best_pose;
  }

  // ------------------- Scan line extraction (with post‑merge filtering)
  // -------------------
  std::vector<LineSegment>
  extractSegmentsFromScan(const std::vector<Point2D> &scan_points) const {
    std::vector<std::vector<Point2D>> raw_segs =
        partitionScanIntoSegments(scan_points);
    std::vector<LineSegment> fitted;
    for (const auto &pts : raw_segs) {
      // (Optional) Keep the distance filter if you like – it's not length‑based
      double mean_range = 0.0;
      for (const auto &p : pts) {
        mean_range += std::sqrt(p.x * p.x + p.y * p.y);
      }
      mean_range /= pts.size();
      if (mean_range > 3.0)
        continue; // ignore far‑away lines

      LineSegment seg;
      if (fitLinePCA(pts, seg)) { // No length check inside fitLinePCA anymore
        fitted.push_back(seg);
      }
    }

    // Merge collinear segments first
    std::vector<LineSegment> merged = mergeCollinearSegments(fitted);

    // Finally, discard any segment shorter than 0.6 m
    std::vector<LineSegment> final_segments;
    for (const auto &seg : merged) {
      if (seg.length >= 0.6) {
        final_segments.push_back(seg);
      }
    }
    return final_segments;
  }

private:
  // ===================== NEW SLIDING‑WINDOW SEGMENTATION =====================
  // Parameters – tune these for your sensor / environment
  // ===================== SLIDING‑WINDOW SEGMENTATION PARAMETERS
  // =====================
  static constexpr size_t WINDOW_SIZE = 10; // points used for local direction
  static constexpr size_t PENDING_LIMIT =
      3; // consecutive deviating points before cutting
  static constexpr double ANGLE_THRESHOLD =
      0.26; // radians (~15°) – corner detection
  static constexpr double LOWPASS_ALPHA =
      0.1; // how fast established_phi follows local
  static constexpr double GAP_THRESHOLD =
      0.30; // 30cm – break on large Euclidean jumps
  std::vector<std::vector<Point2D>>
  partitionScanIntoSegments(const std::vector<Point2D> &scan_points) const {
    std::vector<std::vector<Point2D>> segments;
    if (scan_points.empty())
      return segments;

    // ---------- State ----------
    std::vector<Point2D> current_seg;
    std::vector<size_t> current_seg_indices;
    std::deque<Point2D> window;
    std::vector<Point2D> pending;
    std::vector<size_t> pending_indices;

    std::vector<size_t> segment_start_indices;
    std::vector<size_t> segment_end_indices;

    double established_phi = 0.0;
    bool have_established = false;

    // ---------- Helper: local direction from window ----------
    auto getLocalDirection = [&](const std::deque<Point2D> &win) -> double {
      if (win.size() < 3)
        return 0.0;
      std::vector<Point2D> pts(win.begin(), win.end());
      LineSegment tmp;
      if (!fitLinePCA(pts, tmp))
        return 0.0;
      return tmp.phi;
    };

    // ---------- Helper: finalise current segment ----------
    auto finalizeSegment = [&]() {
      if (current_seg.size() >= 5) {
        segments.push_back(current_seg);
        segment_start_indices.push_back(current_seg_indices.front());
        segment_end_indices.push_back(current_seg_indices.back());
      }
      current_seg.clear();
      current_seg_indices.clear();
      have_established = false;
      pending.clear();
      pending_indices.clear();
      window.clear();
    };

    // ---------- Process each point (original sliding‑window logic) ----------
    for (size_t i = 0; i < scan_points.size(); ++i) {
      const Point2D &p = scan_points[i];

      // ---- 0) Distance breakpoint (Euclidean jump) ----
      if (i > 0) {
        const Point2D &p_prev = scan_points[i - 1];
        double dx = p.x - p_prev.x;
        double dy = p.y - p_prev.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > GAP_THRESHOLD) {
          finalizeSegment();
          window.clear();
          pending.clear();
          pending_indices.clear();
          have_established = false;
          current_seg.push_back(p);
          current_seg_indices.push_back(i);
          window.push_back(p);
          continue;
        }
      }

      // ---- 1) Update sliding window ----
      window.push_back(p);
      if (window.size() > WINDOW_SIZE)
        window.pop_front();

      // ---- 2) Wait for enough points ----
      if (window.size() < WINDOW_SIZE) {
        current_seg.push_back(p);
        current_seg_indices.push_back(i);
        continue;
      }

      // ---- 3) Compute local direction ----
      double local_phi = getLocalDirection(window);
      if (local_phi == 0.0) {
        current_seg.push_back(p);
        current_seg_indices.push_back(i);
        continue;
      }

      // ---- 4) Initialise established direction ----
      if (!have_established) {
        established_phi = local_phi;
        have_established = true;
        continue;
      }

      // ---- 5) Angular difference ----
      double d_phi = std::abs(local_phi - established_phi);
      if (d_phi > M_PI)
        d_phi = 2 * M_PI - d_phi;

      // ---- 6) Decision: aligned or deviated? ----
      if (d_phi < ANGLE_THRESHOLD) {
        // Aligned – re‑attach pending points
        if (!pending.empty()) {
          current_seg.insert(current_seg.end(), pending.begin(), pending.end());
          current_seg_indices.insert(current_seg_indices.end(), pending_indices.begin(), pending_indices.end());
          pending.clear();
          pending_indices.clear();
        }
        current_seg.push_back(p);
        current_seg_indices.push_back(i);
        established_phi =
            (1.0 - LOWPASS_ALPHA) * established_phi + LOWPASS_ALPHA * local_phi;
      } else {
        // Deviated – accumulate pending
        pending.push_back(p);
        pending_indices.push_back(i);

        if (pending.size() >= PENDING_LIMIT) {
          // Real corner – cut here
          finalizeSegment();
          current_seg.insert(current_seg.end(), pending.begin(), pending.end());
          current_seg_indices.insert(current_seg_indices.end(), pending_indices.begin(), pending_indices.end());
          pending.clear();
          pending_indices.clear();
          established_phi = local_phi;
          have_established = true;
          window.clear();
          size_t start = current_seg.size() > WINDOW_SIZE
                             ? current_seg.size() - WINDOW_SIZE
                             : 0;
          for (size_t k = start; k < current_seg.size(); ++k)
            window.push_back(current_seg[k]);
        }
        // else: keep pending, do not add to current_seg
      }
    }

    // ---- End of scan: finalise ----
    if (!pending.empty()) {
      current_seg.insert(current_seg.end(), pending.begin(), pending.end());
      current_seg_indices.insert(current_seg_indices.end(), pending_indices.begin(), pending_indices.end());
      pending.clear();
      pending_indices.clear();
    }
    if (current_seg.size() >= 5) {
      segments.push_back(current_seg);
      segment_start_indices.push_back(current_seg_indices.front());
      segment_end_indices.push_back(current_seg_indices.back());
    }

    // ============================================================
    //   WRAP‑AROUND MERGE WITH COVERAGE CHECK + GAP POINT COLLINEARITY
    // ============================================================
    if (segments.size() >= 2) {
      std::vector<size_t> start_candidates, end_candidates;
      for (size_t i = 0; i < segments.size(); ++i) {
        if (segments[i].size() < 2)
          continue;

        if (segment_start_indices[i] < 15)
          start_candidates.push_back(i);
        if (segment_end_indices[i] > scan_points.size() - 15)
          end_candidates.push_back(i);
      }

      // printf("[WRAP-MERGE] Start candidates: %zu, End candidates: %zu\n",
      //        start_candidates.size(), end_candidates.size());

      std::vector<bool> merged_out(segments.size(), false);
      for (size_t s_idx : start_candidates) {
        if (merged_out[s_idx])
          continue;
        for (size_t e_idx : end_candidates) {
          if (merged_out[e_idx])
            continue;
          if (s_idx == e_idx)
            continue;

          auto &seg_start = segments[s_idx];
          auto &seg_end = segments[e_idx];

          // Fit PCA lines to both start and end segments to get their true projected endpoints (disable quality check)
          LineSegment start_line, end_line;
          bool ok_start = false;
          if (seg_start.size() >= 3) {
            ok_start = fitLinePCA(seg_start, start_line, false);
          } else if (seg_start.size() == 2) {
            fitLine2Points(seg_start[0], seg_start[1], start_line);
            ok_start = true;
          }

          bool ok_end = false;
          if (seg_end.size() >= 3) {
            ok_end = fitLinePCA(seg_end, end_line, false);
          } else if (seg_end.size() == 2) {
            fitLine2Points(seg_end[0], seg_end[1], end_line);
            ok_end = true;
          }

          if (!ok_start || !ok_end) {
            // printf("[WRAP-MERGE]   Skip pair (%zu,%zu): PCA fit failed (ok_start=%d, ok_end=%d)\n", s_idx, e_idx, ok_start, ok_end);
            continue;
          }

          // Check collinearity of the two segments
          double d_phi = std::abs(start_line.phi - end_line.phi);
          if (d_phi > M_PI)
            d_phi = 2 * M_PI - d_phi;
          double d_rho = std::abs(start_line.rho - end_line.rho);

          bool collinear = false;
          if (d_phi < 0.15 && d_rho < 0.15) {
            collinear = true;
          } else if (std::abs(d_phi - M_PI) < 0.15 &&
                     std::abs(start_line.rho + end_line.rho) < 0.15) {
            // Collinear but opposite normals
            collinear = true;
          }

          // Calculate shortest distance between endpoints
          auto dist = [](const Point2D &p1, const Point2D &p2) -> double {
            return std::hypot(p1.x - p2.x, p1.y - p2.y);
          };
          double gap = std::min({
              dist(start_line.p1, end_line.p1),
              dist(start_line.p1, end_line.p2),
              dist(start_line.p2, end_line.p1),
              dist(start_line.p2, end_line.p2)
          });

          // printf("[WRAP-MERGE]   d_phi=%.3f rad (%.1f°), d_rho=%.3f m, collinear=%d, gap=%.3f m\n", d_phi, d_phi * 180.0 / M_PI, d_rho, collinear, gap);

          if (!collinear)
            continue;

          // Check if gap is within 0.3 meters
          if (gap > 0.3) {
            // printf("[WRAP-MERGE]   Collinear pair (%zu,%zu) rejected due to gap (%.3f m > 0.3 m)\n", s_idx, e_idx, gap);
            continue;
          }

          // Merge if they are collinear and close
          std::vector<Point2D> combined = seg_start;
          combined.insert(combined.end(), seg_end.begin(), seg_end.end());

          LineSegment final_seg;
          bool final_ok = fitLinePCA(combined, final_seg, false);
          if (final_ok) {
            double sum_sq = 0.0;
            for (const auto &p : combined) {
              double dist_line = p.x * std::cos(final_seg.phi) +
                                 p.y * std::sin(final_seg.phi) - final_seg.rho;
              sum_sq += dist_line * dist_line;
            }
            double rms = std::sqrt(sum_sq / combined.size());
            if (rms < 0.15) {
              // printf("[WRAP-MERGE]   SUCCESS: Merged segments %zu and %zu (gap=%.3f m, RMS=%.3f, len=%.3f)\n",
              //        s_idx, e_idx, gap, rms, final_seg.length);
              seg_start = combined;
              merged_out[e_idx] = true;
              break;
            } else {
              // printf("[WRAP-MERGE]   Rejected merged fit: RMS=%.3f (>= 0.15)\n", rms);
            }
          }
        }
      }

      // Remove merged segments
      for (int i = (int)segments.size() - 1; i >= 0; --i) {
        if (merged_out[i]) {
          segments.erase(segments.begin() + i);
          segment_start_indices.erase(segment_start_indices.begin() + i);
          segment_end_indices.erase(segment_end_indices.begin() + i);
        }
      }
    }

    return segments;
  }
  // ===================== END NEW SEGMENTATION =====================

  // ------------------- Line fitting (no length rejection) -------------------
  void fitLine2Points(const Point2D &p1, const Point2D &p2,
                      LineSegment &seg) const {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6) {
      seg.phi = 0.0;
      seg.rho = 0.0;
      seg.length = 0.0;
      seg.p1 = p1;
      seg.p2 = p2;
      return;
    }
    double nx = -dy / len;
    double ny = dx / len;
    seg.phi = std::atan2(ny, nx);
    seg.rho = p1.x * nx + p1.y * ny;

    if (p1.x * nx + p1.y * ny > 0.0) {
      nx = -nx;
      ny = -ny;
      seg.phi = std::atan2(ny, nx);
      seg.rho = -seg.rho;
    }

    seg.length = len;
    seg.p1 = p1;
    seg.p2 = p2;
  }

  bool fitLinePCA(const std::vector<Point2D> &points, LineSegment &seg, bool check_quality = true) const {
    if (points.size() < 3)
      return false;

    double sum_x = 0.0, sum_y = 0.0;
    for (const auto &p : points) {
      sum_x += p.x;
      sum_y += p.y;
    }
    double mean_x = sum_x / points.size();
    double mean_y = sum_y / points.size();

    double cov_xx = 0.0, cov_xy = 0.0, cov_yy = 0.0;
    for (const auto &p : points) {
      double dx = p.x - mean_x;
      double dy = p.y - mean_y;
      cov_xx += dx * dx;
      cov_xy += dx * dy;
      cov_yy += dy * dy;
    }

    Eigen::Matrix2d cov;
    cov << cov_xx, cov_xy, cov_xy, cov_yy;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
    double lambda_min = es.eigenvalues()(0);

    Eigen::Vector2d normal = es.eigenvectors().col(0).normalized();
    Eigen::Vector2d tangent = es.eigenvectors().col(1).normalized();

    double phi = std::atan2(normal.y(), normal.x());
    double rho = mean_x * normal.x() + mean_y * normal.y();

    if (mean_x * normal.x() + mean_y * normal.y() > 0.0) {
      normal = -normal;
      phi = std::atan2(normal.y(), normal.x());
      rho = -rho;
    }

    double s_min = std::numeric_limits<double>::max();
    double s_max = std::numeric_limits<double>::lowest();
    for (const auto &p : points) {
      double s = (p.x - mean_x) * tangent.x() + (p.y - mean_y) * tangent.y();
      s_min = std::min(s_min, s);
      s_max = std::max(s_max, s);
    }

    seg.phi = phi;
    seg.rho = rho;
    seg.length = s_max - s_min;
    seg.p1 =
        Point2D{mean_x + s_min * tangent.x(), mean_y + s_min * tangent.y()};
    seg.p2 =
        Point2D{mean_x + s_max * tangent.x(), mean_y + s_max * tangent.y()};

    // Quality check: RMS residual relative to length (keep as a sanity check)
    if (check_quality) {
      double rms = std::sqrt(lambda_min / points.size());
      if (rms / seg.length > 0.10)
        return false;
    }
 
    // NO length filter here – we filter after merging
    return true;
  }

  double computeGap(const LineSegment &a, const LineSegment &b) const {
    double dx = a.p2.x - a.p1.x;
    double dy = a.p2.y - a.p1.y;
    double len_a = std::sqrt(dx * dx + dy * dy);
    if (len_a < 1e-8)
      return std::numeric_limits<double>::max();

    double ux = dx / len_a;
    double uy = dy / len_a;

    double t_b1 = (b.p1.x - a.p1.x) * ux + (b.p1.y - a.p1.y) * uy;
    double t_b2 = (b.p2.x - a.p1.x) * ux + (b.p2.y - a.p1.y) * uy;
    double t_b_min = std::min(t_b1, t_b2);
    double t_b_max = std::max(t_b1, t_b2);

    if (t_b_max < 0.0) {
      return -t_b_max;
    } else if (t_b_min > len_a) {
      return t_b_min - len_a;
    } else {
      return 0.0;
    }
  }

  // ------------------- Merging (unchanged) -------------------
  std::vector<LineSegment>
  mergeCollinearSegments(const std::vector<LineSegment> &segs) const {
    std::vector<LineSegment> merged;
    if (segs.empty())
      return merged;

    std::vector<bool> visited(segs.size(), false);

    for (size_t i = 0; i < segs.size(); ++i) {
      if (visited[i])
        continue;

      std::vector<Point2D> group_pts;
      group_pts.push_back(segs[i].p1);
      group_pts.push_back(segs[i].p2);
      visited[i] = true;

      LineSegment running_seg = segs[i]; // Track the running merged segment

      for (size_t j = i + 1; j < segs.size(); ++j) {
        if (visited[j])
          continue;

        double d_phi = std::abs(running_seg.phi - segs[j].phi);
        if (d_phi > M_PI)
          d_phi = 2 * M_PI - d_phi;

        double d_rho = std::abs(running_seg.rho - segs[j].rho);

        double gap = computeGap(running_seg, segs[j]);

        // printf("[COLLINEAR-DEBUG] Evaluated pair (%zu,%zu): d_phi=%.3f (%.1f°), d_rho=%.3f, gap=%.3f m\n", i, j, d_phi, d_phi * 180.0 / M_PI, d_rho, gap);

        if (d_phi < 0.15 && d_rho < 0.15 && gap < 0.3) {
          // printf("[COLLINEAR-DEBUG]   SUCCESS: merging pair (%zu,%zu)\n", i, j);
          group_pts.push_back(segs[j].p1);
          group_pts.push_back(segs[j].p2);
          visited[j] = true;

          // Update the running segment by fitting to the accumulated points (disable quality check)
          fitLinePCA(group_pts, running_seg, false);
        }
      }

      LineSegment merged_seg;
      if (fitLinePCA(group_pts, merged_seg)) {
        merged.push_back(merged_seg);
      } else {
        merged.push_back(segs[i]);
      }
    }
    return merged;
  }

  // ------------------- Map line extraction (unchanged) -------------------
public:
  std::vector<LineSegment> extractSegmentsFromMap() const {
    std::vector<LineSegment> raw_segs;
    const size_t M = map_points_.size();
    if (M == 0)
      return raw_segs;

    std::vector<bool> visited(M, false);

    for (size_t i = 0; i < M; ++i) {
      if (visited[i])
        continue;

      std::vector<size_t> cluster_indices;
      cluster_indices.push_back(i);
      visited[i] = true;

      size_t second_idx = M;
      double min_d2 = 0.09; // 30cm search
      for (size_t j = 0; j < M; ++j) {
        if (visited[j])
          continue;
        double dx = map_points_[i].x - map_points_[j].x;
        double dy = map_points_[i].y - map_points_[j].y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
          min_d2 = d2;
          second_idx = j;
        }
      }

      if (second_idx == M)
        continue;

      cluster_indices.push_back(second_idx);
      visited[second_idx] = true;

      bool growing = true;
      while (growing) {
        growing = false;

        std::vector<Point2D> current_points;
        for (size_t idx : cluster_indices) {
          current_points.push_back(map_points_[idx]);
        }

        LineSegment temp_seg;
        if (current_points.size() < 3) {
          fitLine2Points(current_points[0], current_points[1], temp_seg);
        } else {
          if (!fitLinePCA(current_points, temp_seg)) {
            break;
          }
        }

        size_t best_next = M;
        double best_dist_line = 0.05; // 5cm line threshold

        for (size_t j = 0; j < M; ++j) {
          if (visited[j])
            continue;

          const auto &pj = map_points_[j];
          double dist_line =
              std::abs(pj.x * std::cos(temp_seg.phi) +
                       pj.y * std::sin(temp_seg.phi) - temp_seg.rho);
          if (dist_line < best_dist_line) {
            double d1_sq = (pj.x - temp_seg.p1.x) * (pj.x - temp_seg.p1.x) +
                           (pj.y - temp_seg.p1.y) * (pj.y - temp_seg.p1.y);
            double d2_sq = (pj.x - temp_seg.p2.x) * (pj.x - temp_seg.p2.x) +
                           (pj.y - temp_seg.p2.y) * (pj.y - temp_seg.p2.y);
            if (d1_sq < 0.09 || d2_sq < 0.09) { // 30cm proximity to endpoints
              best_next = j;
              best_dist_line = dist_line;
            }
          }
        }

        if (best_next != M) {
          cluster_indices.push_back(best_next);
          visited[best_next] = true;
          growing = true;
        }
      }

      if (cluster_indices.size() >= 5) {
        std::vector<Point2D> final_points;
        for (size_t idx : cluster_indices) {
          final_points.push_back(map_points_[idx]);
        }
        LineSegment seg;
        if (fitLinePCA(final_points, seg)) {
          raw_segs.push_back(seg);
        }
      }
    }

    // Merge collinear map segments (just like we do for scans)
    std::vector<LineSegment> merged = mergeCollinearSegments(raw_segs);

    // Filter out segments shorter than 0.6 m
    std::vector<LineSegment> final_map_segs;
    for (const auto &seg : merged) {
      if (seg.length >= 0.6) {
        final_map_segs.push_back(seg);
      }
    }
    return final_map_segs;
  }

private:
  // ------------------- Rotation & translation solvers (unchanged)
  // -------------------
  std::vector<double>
  searchRotation(const std::vector<LineSegment> &scan_segs,
                 const std::vector<LineSegment> &map_segs) const {
    const int num_bins = 360;
    std::vector<double> histogram(num_bins, 0.0);

    for (const auto &ss : scan_segs) {
      for (const auto &ms : map_segs) {
        double theta1 = ms.phi - ss.phi;
        while (theta1 < -M_PI)
          theta1 += 2.0 * M_PI;
        while (theta1 > M_PI)
          theta1 -= 2.0 * M_PI;

        double theta2 = theta1 + M_PI;
        if (theta2 > M_PI)
          theta2 -= 2.0 * M_PI;

        double weight = std::min(ss.length, ms.length);

        // Vote for theta1
        double deg1 = (theta1 + M_PI) * 180.0 / M_PI;
        int bin1 =
            std::clamp(static_cast<int>(std::round(deg1)), 0, num_bins - 1);
        histogram[bin1] += weight;

        // Vote for theta2 (180-degree normal ambiguity)
        double deg2 = (theta2 + M_PI) * 180.0 / M_PI;
        int bin2 =
            std::clamp(static_cast<int>(std::round(deg2)), 0, num_bins - 1);
        histogram[bin2] += weight;
      }
    }

    // Apply moving average smoothing: [0.25, 0.5, 0.25]
    std::vector<double> smoothed(num_bins, 0.0);
    for (int i = 0; i < num_bins; ++i) {
      int prev = (i - 1 + num_bins) % num_bins;
      int next = (i + 1) % num_bins;
      smoothed[i] = 0.25 * histogram[prev] + 0.5 * histogram[i] + 0.25 * histogram[next];
    }

    double max_val = 0.0;
    for (double val : smoothed) {
      max_val = std::max(max_val, val);
    }

    std::vector<double> hypotheses;
    if (max_val < 0.01) {
      hypotheses.push_back(0.0);
      return hypotheses;
    }

    for (int bin = 0; bin < num_bins; ++bin) {
      if (smoothed[bin] >= 0.8 * max_val) {
        int prev_bin = (bin - 1 + num_bins) % num_bins;
        int next_bin = (bin + 1) % num_bins;
        if (smoothed[bin] >= smoothed[prev_bin] &&
            smoothed[bin] >= smoothed[next_bin]) {
          
          // Sub-bin parabolic interpolation:
          double y_prev = smoothed[prev_bin];
          double y_curr = smoothed[bin];
          double y_next = smoothed[next_bin];

          double denom = 2.0 * (y_prev - 2.0 * y_curr + y_next);
          double offset = 0.0;
          if (std::abs(denom) > 1e-5) {
            offset = (y_prev - y_next) / denom;
          }
          offset = std::clamp(offset, -0.5, 0.5);

          double peak_deg = static_cast<double>(bin) + offset;
          double theta = (peak_deg / 180.0 * M_PI) - M_PI;
          
          while (theta < -M_PI) theta += 2.0 * M_PI;
          while (theta > M_PI)  theta -= 2.0 * M_PI;

          hypotheses.push_back(theta);
        }
      }
    }
    return hypotheses;
  }

  std::vector<Pose2D>
  solveTranslation(const std::vector<LineSegment> &scan_segs,
                   const std::vector<LineSegment> &map_segs,
                   const std::vector<double> &theta_hypotheses) const {
    std::vector<Pose2D> candidates;

    for (double theta : theta_hypotheses) {
      std::vector<LineMatch> matches;
      for (const auto &ss : scan_segs) {
        double ss_phi_rot = ss.phi + theta;
        while (ss_phi_rot < -M_PI)
          ss_phi_rot += 2 * M_PI;
        while (ss_phi_rot > M_PI)
          ss_phi_rot -= 2 * M_PI;

        for (const auto &ms : map_segs) {
          double d_phi = std::abs(ms.phi - ss_phi_rot);
          if (d_phi > M_PI)
            d_phi = 2 * M_PI - d_phi;

          if (d_phi < 0.26) {
            matches.push_back({ms.phi, ms.rho, ss.rho});
          } else if (std::abs(d_phi - M_PI) < 0.26) {
            matches.push_back({ms.phi, ms.rho, -ss.rho});
          }
        }
      }

      if (matches.size() >= 2) {
        Eigen::MatrixXd A(matches.size(), 2);
        Eigen::VectorXd B(matches.size());
        for (size_t i = 0; i < matches.size(); ++i) {
          A(i, 0) = std::cos(matches[i].map_phi);
          A(i, 1) = std::sin(matches[i].map_phi);
          B(i) = matches[i].map_rho - matches[i].scan_rho;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU |
                                                     Eigen::ComputeThinV);
        Eigen::Vector2d X_Y = svd.solve(B);

        Pose2D p;
        p.x = X_Y(0);
        p.y = X_Y(1);
        p.theta = theta;
        candidates.push_back(p);
      }
    }
    return candidates;
  }

private:
  std::vector<Point2D> map_points_;
  std::vector<LineSegment> map_segs_;
};

} // namespace icp2d