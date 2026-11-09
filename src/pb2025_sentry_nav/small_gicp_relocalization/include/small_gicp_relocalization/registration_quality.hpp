// Copyright 2026 diting
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__REGISTRATION_QUALITY_HPP_
#define SMALL_GICP_RELOCALIZATION__REGISTRATION_QUALITY_HPP_

#include <cmath>
#include <cstddef>

namespace small_gicp_relocalization
{

// These are commissioning thresholds, not a proof of global localization or robot safety.
struct RegistrationQualityLimits
{
  std::size_t min_inliers = 100;
  double min_inlier_ratio = 0.2;
  double max_mean_error = 5.0;  // covariance-weighted cost, not a distance in metres
  double max_translation_step = 0.75;
  double max_rotation_step = 0.6;
};

inline const char * registrationRejection(
  bool converged, bool finite, std::size_t inliers, std::size_t points, double error,
  double translation_step, double rotation_step, const RegistrationQualityLimits & limits)
{
  if (!converged) {
    return "not_converged";
  }
  if (
    !finite || !std::isfinite(error) || error < 0.0 || !std::isfinite(translation_step) ||
    !std::isfinite(rotation_step)) {
    return "non_finite_result";
  }
  if (points == 0 || inliers < limits.min_inliers || inliers > points) {
    return "insufficient_inliers";
  }
  if (static_cast<double>(inliers) / points < limits.min_inlier_ratio) {
    return "low_inlier_ratio";
  }
  if (error / inliers > limits.max_mean_error) {
    return "high_mean_gicp_error";
  }
  if (translation_step > limits.max_translation_step || rotation_step > limits.max_rotation_step) {
    return "registration_jump";
  }
  return nullptr;
}

}  // namespace small_gicp_relocalization
#endif  // SMALL_GICP_RELOCALIZATION__REGISTRATION_QUALITY_HPP_
