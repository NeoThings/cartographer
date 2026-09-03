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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_WALL_DIRECTION_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_WALL_DIRECTION_H_

#include "absl/types/optional.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

namespace cartographer {
namespace mapping {

// Estimates an initial yaw (radians about +Z) for the pose extrapolator so
// that the dominant wall (RANSAC) is parallel to the nearest map axis among
// {+x, -x, +y, -y}. Returns nullopt if no reliable wall is found.
absl::optional<double> EstimateInitialYawToAlignWalls(
    const sensor::TimedPointCloudOriginData& range_data, float min_range,
    float max_range, float min_z, float max_z);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_WALL_DIRECTION_H_
