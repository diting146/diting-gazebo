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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__LINEAR_SPEED_PROFILE_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__LINEAR_SPEED_PROFILE_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pb_omni_pid_pursuit_controller
{

// Regulate soft speed changes in both directions. Regional/configured bounds
// remain hard caps; collision rejection and action stops bypass this profile.
class LinearSpeedProfile
{
public:
  void reset(double speed = 0.0) { speed_ = std::isfinite(speed) ? speed : 0.0; }

  double update(double target, double limit, double rate, double dt)
  {
    if (
      !std::isfinite(target) || !std::isfinite(limit) || limit < 0.0 || !std::isfinite(rate) ||
      rate <= 0.0 || !std::isfinite(dt) || dt <= 0.0) {
      throw std::invalid_argument("invalid linear speed profile input");
    }
    // A newly tightened speed limit takes effect before acceleration smoothing.
    speed_ = std::clamp(speed_, -limit, limit);
    target = std::clamp(target, -limit, limit);
    speed_ += std::clamp(target - speed_, -rate * dt, rate * dt);
    return speed_;
  }

private:
  double speed_{0.0};
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__LINEAR_SPEED_PROFILE_HPP_
