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

#include "cartographer/mapping/pose_graph_runtime_options.h"

#include <sstream>
#include <string>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

bool ParseBool(const std::string& value, bool* out, std::string* error) {
  if (value == "true" || value == "1") {
    *out = true;
    return true;
  }
  if (value == "false" || value == "0") {
    *out = false;
    return true;
  }
  *error = absl::StrCat("Invalid bool value: '", value, "'");
  return false;
}

bool ParseDouble(const std::string& value, double* out, std::string* error) {
  try {
    size_t idx = 0;
    *out = std::stod(value, &idx);
    if (idx != value.size()) {
      *error = absl::StrCat("Invalid double value: '", value, "'");
      return false;
    }
    return true;
  } catch (const std::exception&) {
    *error = absl::StrCat("Invalid double value: '", value, "'");
    return false;
  }
}

bool ParseInt(const std::string& value, int* out, std::string* error) {
  try {
    size_t idx = 0;
    *out = std::stoi(value, &idx);
    if (idx != value.size()) {
      *error = absl::StrCat("Invalid int value: '", value, "'");
      return false;
    }
    return true;
  } catch (const std::exception&) {
    *error = absl::StrCat("Invalid int value: '", value, "'");
    return false;
  }
}

bool ApplyOneOption(const std::string& name, const std::string& value,
                    proto::PoseGraphOptions* options, std::string* error) {
  auto* constraint_builder = options->mutable_constraint_builder_options();
  auto* optimization_problem = options->mutable_optimization_problem_options();

  if (name == "optimize_every_n_nodes") {
    int v;
    if (!ParseInt(value, &v, error)) return false;
    options->set_optimize_every_n_nodes(v);
    return true;
  }
  if (name == "matcher_translation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    options->set_matcher_translation_weight(v);
    return true;
  }
  if (name == "matcher_rotation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    options->set_matcher_rotation_weight(v);
    return true;
  }
  if (name == "max_num_final_iterations") {
    int v;
    if (!ParseInt(value, &v, error)) return false;
    options->set_max_num_final_iterations(v);
    return true;
  }
  if (name == "global_sampling_ratio") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    options->set_global_sampling_ratio(v);
    return true;
  }
  if (name == "log_residual_histograms") {
    bool v;
    if (!ParseBool(value, &v, error)) return false;
    options->set_log_residual_histograms(v);
    return true;
  }
  if (name == "global_constraint_search_after_n_seconds") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    options->set_global_constraint_search_after_n_seconds(v);
    return true;
  }
  if (name == "optimization_on_first_node") {
    bool v;
    if (!ParseBool(value, &v, error)) return false;
    options->set_optimization_on_first_node(v);
    return true;
  }

  if (name == "constraint_builder.sampling_ratio") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    constraint_builder->set_sampling_ratio(v);
    return true;
  }
  if (name == "constraint_builder.max_constraint_distance") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    constraint_builder->set_max_constraint_distance(v);
    return true;
  }
  if (name == "constraint_builder.max_constraint_candidates") {
    int v;
    if (!ParseInt(value, &v, error)) return false;
    constraint_builder->set_max_constraint_candidates(v);
    return true;
  }
  if (name == "constraint_builder.limit_global_constraint_distance") {
    bool v;
    if (!ParseBool(value, &v, error)) return false;
    constraint_builder->set_limit_global_constraint_distance(v);
    return true;
  }
  if (name == "constraint_builder.min_score") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    constraint_builder->set_min_score(v);
    return true;
  }
  if (name == "constraint_builder.global_localization_min_score") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    constraint_builder->set_global_localization_min_score(v);
    return true;
  }
  if (name == "constraint_builder.loop_closure_translation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    constraint_builder->set_loop_closure_translation_weight(v);
    return true;
  }
  if (name == "constraint_builder.loop_closure_rotation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    constraint_builder->set_loop_closure_rotation_weight(v);
    return true;
  }
  if (name == "constraint_builder.log_matches") {
    bool v;
    if (!ParseBool(value, &v, error)) return false;
    constraint_builder->set_log_matches(v);
    return true;
  }
  if (name == "constraint_builder.log_constraint_search") {
    bool v;
    if (!ParseBool(value, &v, error)) return false;
    constraint_builder->set_log_constraint_search(v);
    return true;
  }

  if (name == "optimization_problem.huber_scale") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_huber_scale(v);
    return true;
  }
  if (name == "optimization_problem.acceleration_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_acceleration_weight(v);
    return true;
  }
  if (name == "optimization_problem.rotation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_rotation_weight(v);
    return true;
  }
  if (name == "optimization_problem.local_slam_pose_translation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_local_slam_pose_translation_weight(v);
    return true;
  }
  if (name == "optimization_problem.local_slam_pose_rotation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_local_slam_pose_rotation_weight(v);
    return true;
  }
  if (name == "optimization_problem.odometry_translation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_odometry_translation_weight(v);
    return true;
  }
  if (name == "optimization_problem.odometry_rotation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_odometry_rotation_weight(v);
    return true;
  }
  if (name == "optimization_problem.fixed_frame_pose_translation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_fixed_frame_pose_translation_weight(v);
    return true;
  }
  if (name == "optimization_problem.fixed_frame_pose_rotation_weight") {
    double v;
    if (!ParseDouble(value, &v, error)) return false;
    optimization_problem->set_fixed_frame_pose_rotation_weight(v);
    return true;
  }
  if (name == "optimization_problem.log_solver_summary") {
    bool v;
    if (!ParseBool(value, &v, error)) return false;
    optimization_problem->set_log_solver_summary(v);
    return true;
  }

  if (name.find("constraint_builder.fast_correlative_scan_matcher") == 0 ||
      name.find("constraint_builder.ceres_scan_matcher") == 0) {
    *error = absl::StrCat(
        "Option '", name,
        "' is baked into scan matchers and cannot be changed at runtime. "
        "Restart the node (or rebuild MapBuilder) to change it.");
    return false;
  }

  *error = absl::StrCat("Unsupported pose graph option: '", name, "'");
  return false;
}

}  // namespace

std::string ApplyPoseGraphRuntimeOptions(
    const std::vector<std::pair<std::string, std::string>>& name_value_pairs,
    proto::PoseGraphOptions* options) {
  CHECK(options != nullptr);
  if (name_value_pairs.empty()) {
    return "No options provided.";
  }
  for (const auto& pair : name_value_pairs) {
    std::string error;
    if (!ApplyOneOption(pair.first, pair.second, options, &error)) {
      return error;
    }
  }
  return "";
}

}  // namespace mapping
}  // namespace cartographer
