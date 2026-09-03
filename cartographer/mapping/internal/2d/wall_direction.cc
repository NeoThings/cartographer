/*
 * Copyright 2026 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/wall_direction.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "cartographer/common/math.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr int kMinNumPoints = 30;
constexpr int kRansacIterations = 300;
constexpr double kInlierDistanceMeters = 0.05;
constexpr int kMinInliers = 25;
constexpr double kMinLineLengthMeters = 0.8;
constexpr double kMinSampleSeparationMeters = 0.2;

struct LineModel {
  Eigen::Vector2d point = Eigen::Vector2d::Zero();
  Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
};

double PointToLineDistance(const Eigen::Vector2d& p, const LineModel& line) {
  const Eigen::Vector2d delta = p - line.point;
  return std::abs(delta.x() * line.direction.y() - delta.y() * line.direction.x());
}

LineModel FitLine(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
  LineModel line;
  line.point = a;
  line.direction = (b - a).normalized();
  return line;
}

// Refine line direction from inliers via 2D PCA (along-wall = largest variance).
absl::optional<LineModel> RefineLineFromInliers(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& inlier_indices) {
  if (inlier_indices.size() < 2) {
    return absl::nullopt;
  }
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const int index : inlier_indices) {
    mean += points[index];
  }
  mean /= static_cast<double>(inlier_indices.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const int index : inlier_indices) {
    const Eigen::Vector2d delta = points[index] - mean;
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<double>(inlier_indices.size());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return absl::nullopt;
  }
  LineModel line;
  line.point = mean;
  line.direction = solver.eigenvectors().col(1).normalized();
  return line;
}

double InlierSpanAlongLine(const std::vector<Eigen::Vector2d>& points,
                           const std::vector<int>& inlier_indices,
                           const LineModel& line) {
  double min_proj = std::numeric_limits<double>::infinity();
  double max_proj = -std::numeric_limits<double>::infinity();
  for (const int index : inlier_indices) {
    const double proj = (points[index] - line.point).dot(line.direction);
    min_proj = std::min(min_proj, proj);
    max_proj = std::max(max_proj, proj);
  }
  return max_proj - min_proj;
}

// Fold undirected wall angle to the nearest of {±x, ±y}, return extrapolator yaw.
// Candidate axis angles: 0, ±pi/2, pi (same as -pi). Smallest |delta| wins; yaw = -delta.
double YawToAlignNearestAxis(double wall_angle) {
  wall_angle = common::NormalizeAngleDifference(wall_angle);
  // Undirected line: theta ~ theta+pi.
  double delta = wall_angle;
  constexpr double kQuarterPi = M_PI / 4.;
  while (delta > kQuarterPi) {
    delta -= M_PI / 2.;
  }
  while (delta <= -kQuarterPi) {
    delta += M_PI / 2.;
  }
  return -delta;
}

absl::optional<double> EstimateYawFromLineInliers(
    const std::vector<Eigen::Vector2d>& points) {
  if (static_cast<int>(points.size()) < kMinNumPoints) {
    return absl::nullopt;
  }

  std::mt19937 rng(42);
  std::uniform_int_distribution<int> index_dist(
      0, static_cast<int>(points.size()) - 1);

  int best_inlier_count = 0;
  double best_span = 0.;
  std::vector<int> best_inliers;

  for (int iteration = 0; iteration < kRansacIterations; ++iteration) {
    const int i = index_dist(rng);
    const int j = index_dist(rng);
    if (i == j) {
      continue;
    }
    if ((points[i] - points[j]).norm() < kMinSampleSeparationMeters) {
      continue;
    }
    const LineModel candidate = FitLine(points[i], points[j]);
    std::vector<int> inliers;
    inliers.reserve(points.size());
    for (int k = 0; k < static_cast<int>(points.size()); ++k) {
      if (PointToLineDistance(points[k], candidate) <= kInlierDistanceMeters) {
        inliers.push_back(k);
      }
    }
    if (static_cast<int>(inliers.size()) < kMinInliers) {
      continue;
    }
    const double span = InlierSpanAlongLine(points, inliers, candidate);
    if (span < kMinLineLengthMeters) {
      continue;
    }
    if (static_cast<int>(inliers.size()) > best_inlier_count ||
        (static_cast<int>(inliers.size()) == best_inlier_count &&
         span > best_span)) {
      best_inlier_count = static_cast<int>(inliers.size());
      best_span = span;
      best_inliers = std::move(inliers);
    }
  }

  if (best_inliers.empty()) {
    LOG(WARNING) << "align_to_wall: RANSAC found no wall line "
                    "(need >= "
                 << kMinInliers << " inliers, length >= "
                 << kMinLineLengthMeters << " m).";
    return absl::nullopt;
  }

  const absl::optional<LineModel> refined =
      RefineLineFromInliers(points, best_inliers);
  if (!refined.has_value()) {
    LOG(WARNING) << "align_to_wall: failed to refine wall line.";
    return absl::nullopt;
  }

  const double wall_angle =
      std::atan2(refined->direction.y(), refined->direction.x());
  const double yaw = YawToAlignNearestAxis(wall_angle);
  LOG(INFO) << "align_to_wall: RANSAC wall angle (rad) = " << wall_angle
            << ", inliers = " << best_inlier_count << ", length = " << best_span
            << " m, initial yaw (rad) = " << yaw;
  return yaw;
}

}  // namespace

absl::optional<double> EstimateInitialYawToAlignWalls(
    const sensor::TimedPointCloudOriginData& range_data, const float min_range,
    const float max_range, const float min_z, const float max_z) {
  std::vector<Eigen::Vector2d> points;
  points.reserve(range_data.ranges.size());
  for (const auto& measurement : range_data.ranges) {
    const Eigen::Vector3f& hit = measurement.point_time.position;
    if (hit.z() < min_z || hit.z() > max_z) {
      continue;
    }
    const Eigen::Vector3f& origin =
        range_data.origins.at(measurement.origin_index);
    const float range = (hit - origin).norm();
    if (range < min_range || range > max_range) {
      continue;
    }
    points.emplace_back(hit.x(), hit.y());
  }
  if (static_cast<int>(points.size()) < kMinNumPoints) {
    LOG(WARNING) << "align_to_wall: not enough points for line extraction ("
                 << points.size() << " < " << kMinNumPoints << ").";
    return absl::nullopt;
  }
  return EstimateYawFromLineInliers(points);
}

}  // namespace mapping
}  // namespace cartographer
