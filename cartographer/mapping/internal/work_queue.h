/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H
#define CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H

#include <chrono>
#include <deque>
#include <functional>

namespace cartographer {
namespace mapping {

struct WorkItem {
  enum class Result {
    kDoNotRunOptimization,
    kRunOptimization,
  };

  enum class Type {
    kAddData,
    kComputeConstraint,
    kRunOptimization,
  };

  std::chrono::steady_clock::time_point time;
  std::function<Result()> task;
  Type type = Type::kAddData;
};

using WorkQueue = std::deque<WorkItem>;

struct WorkQueueCounts {
  size_t add_data = 0;
  size_t compute_constraint = 0;
  size_t run_optimization = 0;
};

inline WorkQueueCounts CountWorkQueueByType(const WorkQueue& queue) {
  WorkQueueCounts counts;
  for (const WorkItem& item : queue) {
    switch (item.type) {
      case WorkItem::Type::kAddData:
        ++counts.add_data;
        break;
      case WorkItem::Type::kComputeConstraint:
        ++counts.compute_constraint;
        break;
      case WorkItem::Type::kRunOptimization:
        ++counts.run_optimization;
        break;
    }
  }
  return counts;
}

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H
