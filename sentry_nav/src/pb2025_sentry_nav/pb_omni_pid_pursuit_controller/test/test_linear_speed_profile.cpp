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

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "pb_omni_pid_pursuit_controller/linear_speed_profile.hpp"

namespace pb_omni_pid_pursuit_controller
{

TEST(LinearSpeedProfile, BendRecoveryAndNewReferencesCannotBypassTheRateLimit)
{
  LinearSpeedProfile profile;
  profile.reset(0.5);
  double previous = 0.5;
  // Reproduce bend/straight switches and the PID reference jumps from the trace.
  for (double target : {1.0, 0.7, 0.35, 0.058, 0.7, 1.0, 0.5}) {
    const double speed = profile.update(target, 1.0, 0.9, 0.05);
    EXPECT_LE(std::abs(speed - previous), 0.045 + 1e-12);
    EXPECT_GE(speed, 0.0);
    EXPECT_LE(speed, 1.0);
    previous = speed;
  }
}

TEST(LinearSpeedProfile, RegionalLimitTightensImmediatelyAndReleaseRampsUp)
{
  LinearSpeedProfile profile;
  profile.reset(1.0);
  EXPECT_DOUBLE_EQ(profile.update(1.0, 0.45, 0.9, 0.05), 0.45);
  EXPECT_NEAR(profile.update(1.0, 1.0, 0.9, 0.05), 0.495, 1e-12);
  EXPECT_DOUBLE_EQ(profile.update(1.0, 0.0, 0.9, 0.05), 0.0);
}

TEST(LinearSpeedProfile, SettlesWithoutOvershootAndCanResetAfterAStop)
{
  LinearSpeedProfile profile;
  for (int tick = 0; tick < 100; ++tick) {
    EXPECT_LE(profile.update(0.8, 1.0, 0.9, 0.05), 0.8);
  }
  EXPECT_DOUBLE_EQ(profile.update(0.8, 1.0, 0.9, 0.05), 0.8);
  profile.reset();
  EXPECT_NEAR(profile.update(0.8, 1.0, 0.9, 0.05), 0.045, 1e-12);
  profile.reset(0.2);
  EXPECT_NEAR(profile.update(0.8, 1.0, 0.9, 0.05), 0.245, 1e-12);
}

TEST(LinearSpeedProfile, InvalidInputsAreRejectedWithoutChangingThePreviousSpeed)
{
  LinearSpeedProfile profile;
  profile.reset(0.5);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(profile.update(nan, 1, 0.9, 0.05), std::invalid_argument);
  EXPECT_THROW(profile.update(1, nan, 0.9, 0.05), std::invalid_argument);
  EXPECT_THROW(profile.update(1, -1, 0.9, 0.05), std::invalid_argument);
  EXPECT_THROW(profile.update(1, 1, 0, 0.05), std::invalid_argument);
  EXPECT_THROW(profile.update(1, 1, 0.9, 0), std::invalid_argument);
  EXPECT_NEAR(profile.update(1, 1, 0.9, 0.05), 0.545, 1e-12);
}

}  // namespace pb_omni_pid_pursuit_controller
