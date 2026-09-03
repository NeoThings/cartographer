/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kNumSubmapScanMatchersMetric = metrics::Gauge::Null();

transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

ConstraintBuilder2D::ConstraintBuilder2D(
    const constraints::proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(absl::make_unique<common::Task>()),
      when_done_task_(absl::make_unique<common::Task>()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()) {}

ConstraintBuilder2D::~ConstraintBuilder2D() {
  absl::MutexLock locker(&mutex_);
  if (shutdown_) {
    // Fast shutdown: skip assertions
    constraints_.clear();
    when_done_.reset();
    // CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
    // CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
    // CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
    // CHECK_EQ(num_started_nodes_, num_finished_nodes_);
    // CHECK(when_done_ == nullptr);
    return;
  }
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder2D::MaybeAddConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool enhance_search,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  if (shutdown_) return;
  if (initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }

  absl::MutexLock locker(&mutex_);
  // re-check after got locker
  if (shutdown_) return;
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  if (!scan_matcher) {
    constraints_.pop_back();
    return;
  }
  num_pending_constraint_computations_.fetch_add(1);
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    // re-check while the work is actually in progress
    if (!shutdown_) {
      ComputeConstraint(submap_id, submap, node_id, false, /* match_full_submap */
                        enhance_search,
                        constant_data, initial_relative_pose, *scan_matcher,
                        constraint);
    }
    num_pending_constraint_computations_.fetch_sub(1);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose) {
  if (shutdown_) return;
  if (options_.limit_global_constraint_distance()) {
    const double distance = initial_relative_pose.translation().norm();
    if (distance > options_.max_constraint_distance()) {
      return;
    }
  }
  absl::MutexLock locker(&mutex_);
  // re-check while the work is actually in progress
  if (shutdown_) return;
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  if (!scan_matcher) {
    constraints_.pop_back();
    return;
  }
  num_pending_constraint_computations_.fetch_add(1);
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    // re-check while the work is actually in progress
    if (!shutdown_) {
      ComputeConstraint(submap_id, submap, node_id, true, /* match_full_submap */
                        false, /* match full submap don't need enhance */
                        constant_data, transform::Rigid2d::Identity(),
                        *scan_matcher, constraint);
    }
    num_pending_constraint_computations_.fetch_sub(1);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::NotifyEndOfNode() {
  absl::MutexLock locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    absl::MutexLock locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = absl::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}

void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  absl::MutexLock locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = absl::make_unique<common::Task>();
}

const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,
                                                     const Grid2D* const grid) {
  CHECK(grid);
  if (shutdown_) return nullptr;
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
  submap_scan_matcher.grid = grid;
  auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options();
  auto scan_matcher_task = absl::make_unique<common::Task>();
  scan_matcher_task->SetWorkItem(
      [this, &submap_scan_matcher, &scan_matcher_options]() {
        // re-check while the work is actually in progress
        if (shutdown_) return;
        submap_scan_matcher.fast_correlative_scan_matcher =
            absl::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                *submap_scan_matcher.grid, scan_matcher_options);
      });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  return &submap_scan_matchers_.at(submap_id);
}

void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, bool match_full_submap,
    bool enhance_match,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;

  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();

  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  if (match_full_submap) {
    kGlobalConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->MatchFullSubmap(
            constant_data->filtered_gravity_aligned_point_cloud,
            options_.global_localization_min_score(), &score, &pose_estimate)) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);
      CHECK_GE(submap_id.trajectory_id, 0);
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
      ++num_global_match_succeeded_;
    } else {
      ++num_global_match_failed_;
      return;
    }
  } else {
    kConstraintsSearchedMetric->Increment();
    if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
            initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
            options_.min_score(), &score, &pose_estimate, enhance_match)) {
      // We've reported a successful local match.
      CHECK_GT(score, options_.min_score());
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
      ++num_local_match_succeeded_;
    } else {
      ++num_local_match_failed_;
      return;
    }
  }
  {
    absl::MutexLock locker(&mutex_);
    score_histogram_.Add(score);
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                            constant_data->filtered_gravity_aligned_point_cloud,
                            *submap_scan_matcher.grid, &pose_estimate,
                            &unused_summary);

  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;

  // Loop-closure error vs the current graph:
  // zbar_ij^{-1} * (T_global_submap^{-1} * T_global_node).
  const transform::Rigid2d delta =
      constraint_transform.inverse() * initial_relative_pose;
  const double trans = delta.translation().norm();
  const double rot = std::abs(delta.normalized_angle());
  const bool reject = node_id.node_index != 0 &&
                      ((options_.max_loop_closure_translation_error() > 0. &&
                        trans > options_.max_loop_closure_translation_error()) ||
                       (options_.max_loop_closure_rotation_error() > 0. &&
                        rot > options_.max_loop_closure_rotation_error()));
  if (reject) {
    LOG(INFO) << "Loop closure error n=" << node_id << " s=" << submap_id
              << std::fixed << std::setprecision(2) << " " << trans << "m "
              << common::RadToDeg(delta.normalized_angle()) << "deg "
              << std::setprecision(0) << 100. * score << "% REJECTED";
    return;
  }

  constraint->reset(new Constraint{submap_id,
                                   node_id,
                                   {transform::Embed3D(constraint_transform),
                                    options_.loop_closure_translation_weight(),
                                    options_.loop_closure_rotation_weight()},
                                   Constraint::INTER_SUBMAP});

  if (options_.log_matches()) {
    // skip the constraint between the same trajectory
    // if(node_id.trajectory_id == submap_id.trajectory_id) {
    //   return;
    // }
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (match_full_submap) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder2D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
    kQueueLengthMetric->Set(constraints_.size());
  }
  if (options_.log_constraint_search()) {
    const int local_fail = num_local_match_failed_.exchange(0);
    const int global_fail = num_global_match_failed_.exchange(0);
    const int local_ok = num_local_match_succeeded_.exchange(0);
    const int global_ok = num_global_match_succeeded_.exchange(0);
    LOG(INFO) << "\n [Debug] Constraint search summary: \n"
              << " local_ok/fail=" << local_ok << "/" << local_fail
              << " global_ok/fail=" << global_ok << "/" << global_fail
              << " result_constraints=" << result.size();
  }
  (*callback)(result);
}

int ConstraintBuilder2D::GetNumFinishedNodes() {
  absl::MutexLock locker(&mutex_);
  return num_finished_nodes_;
}

int ConstraintBuilder2D::GetNumPendingConstraintComputations() {
  return num_pending_constraint_computations_.load();
}

void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
}

void ConstraintBuilder2D::SetOptions(
    const proto::ConstraintBuilderOptions& options) {
  absl::MutexLock locker(&mutex_);
  // Keep existing scan matcher instances; only thresholds/weights/flags change.
  options_.set_sampling_ratio(options.sampling_ratio());
  options_.set_max_constraint_distance(options.max_constraint_distance());
  options_.set_max_constraint_candidates(options.max_constraint_candidates());
  options_.set_limit_global_constraint_distance(
      options.limit_global_constraint_distance());
  options_.set_min_score(options.min_score());
  options_.set_global_localization_min_score(
      options.global_localization_min_score());
  options_.set_loop_closure_translation_weight(
      options.loop_closure_translation_weight());
  options_.set_loop_closure_rotation_weight(
      options.loop_closure_rotation_weight());
  options_.set_log_matches(options.log_matches());
  options_.set_log_constraint_search(options.log_constraint_search());
  options_.set_max_loop_closure_translation_error(options.max_loop_closure_translation_error());
  options_.set_max_loop_closure_rotation_error(options.max_loop_closure_rotation_error());
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_constraints_constraint_builder_2d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_queue_length", "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_constraints_constraint_builder_2d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
  auto* num_matchers = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_num_submap_scan_matchers",
      "Current number of constructed submap scan matchers");
  kNumSubmapScanMatchersMetric = num_matchers->Add({});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
