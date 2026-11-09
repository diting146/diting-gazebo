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

#include <gtest/gtest.h>

#include <limits>

#include "small_gicp_relocalization/registration_quality.hpp"

using small_gicp_relocalization::RegistrationQualityLimits;
using small_gicp_relocalization::registrationRejection;

TEST(RegistrationQuality, AcceptsWellSupportedFiniteConvergence)
{
  RegistrationQualityLimits limits;
  EXPECT_EQ(registrationRejection(true, true, 200, 500, 300., .1, .05, limits), nullptr);
}

TEST(RegistrationQuality, ConvergenceAloneCannotMakeLocalizationTrusted)
{
  RegistrationQualityLimits limits;
  EXPECT_STREQ(
    registrationRejection(false, true, 200, 500, 300., .1, .05, limits), "not_converged");
  EXPECT_STREQ(
    registrationRejection(true, false, 200, 500, 300., .1, .05, limits), "non_finite_result");
  EXPECT_STREQ(
    registrationRejection(true, true, 50, 500, 300., .1, .05, limits), "insufficient_inliers");
  EXPECT_STREQ(
    registrationRejection(true, true, 100, 1000, 300., .1, .05, limits), "low_inlier_ratio");
}

TEST(RegistrationQuality, RejectsLargeErrorAndMapTransformJumps)
{
  RegistrationQualityLimits limits;
  EXPECT_STREQ(
    registrationRejection(true, true, 200, 500, 1100., .1, .05, limits), "high_mean_gicp_error");
  EXPECT_STREQ(
    registrationRejection(true, true, 200, 500, 300., .76, .05, limits), "registration_jump");
  EXPECT_STREQ(
    registrationRejection(true, true, 200, 500, 300., .1, .61, limits), "registration_jump");
}

TEST(RegistrationQuality, RejectsNonFiniteAndImpossibleMetrics)
{
  RegistrationQualityLimits limits;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_STREQ(
    registrationRejection(true, true, 200, 500, nan, .1, .05, limits), "non_finite_result");
  EXPECT_STREQ(
    registrationRejection(true, true, 200, 500, 300., nan, .05, limits), "non_finite_result");
  EXPECT_STREQ(
    registrationRejection(true, true, 200, 0, 300., .1, .05, limits), "insufficient_inliers");
  EXPECT_STREQ(
    registrationRejection(true, true, 501, 500, 300., .1, .05, limits), "insufficient_inliers");
}

TEST(RegistrationQuality, ThresholdsAreExplicitAndConfigurable)
{
  RegistrationQualityLimits limits;
  limits.max_mean_error = 1.;
  EXPECT_STREQ(
    registrationRejection(true, true, 200, 500, 300., .1, .05, limits), "high_mean_gicp_error");
  limits.max_mean_error = 2.;
  EXPECT_EQ(registrationRejection(true, true, 200, 500, 300., .1, .05, limits), nullptr);
}
