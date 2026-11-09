// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#include "sentry_tunnel_bt/passage_diagnostics.hpp"
#include <gtest/gtest.h>
#include <limits>
using sentry_tunnel_bt::passageDeviation;
TEST(PassageDeviation, IdentifiesYawOnly) {
  const auto check = passageDeviation(0.06, 0.0, 3.475, 3.475, 0.0523598775);
  EXPECT_TRUE(check.valid);
  EXPECT_TRUE(check.yaw_exceeded);
  EXPECT_FALSE(check.lateral_exceeded);
  EXPECT_TRUE(check.exceeded());
}
TEST(PassageDeviation, IdentifiesSignedLateralOnly) {
  const auto check = passageDeviation(0.0, 0.0, 3.405, 3.475, 0.0523598775);
  EXPECT_TRUE(check.valid);
  EXPECT_FALSE(check.yaw_exceeded);
  EXPECT_TRUE(check.lateral_exceeded);
  EXPECT_NEAR(check.signed_lateral_error, -0.07, 1e-12);
}
TEST(PassageDeviation, IdentifiesBothAndReverseHeadingWrap) {
  const auto check =
      passageDeviation(-M_PI + 0.06, M_PI, 0.07, 0.0, 0.0523598775);
  EXPECT_TRUE(check.yaw_exceeded);
  EXPECT_TRUE(check.lateral_exceeded);
  EXPECT_NEAR(check.yaw_error, 0.06, 1e-12);
}
TEST(PassageDeviation, StableAndExactLateralBoundaryPreserveStrictComparison) {
  EXPECT_FALSE(
      passageDeviation(M_PI, -M_PI, 0.0, 0.0, 0.0523598775).exceeded());
  for (const double sign : {-1.0, 1.0}) {
    const auto boundary = passageDeviation(0.0, 0.0, sign * 0.06, 0.0, 0.0523598775);
    EXPECT_FALSE(boundary.exceeded());
    EXPECT_TRUE(passageDeviation(0.0, 0.0, sign * 0.060001, 0.0,
                                 0.0523598775).lateral_exceeded);
  }
  const double exact_yaw_limit =
      std::abs(std::atan2(std::sin(-0.06), std::cos(-0.06)));
  const auto yaw_boundary =
      passageDeviation(0.06, 0.0, 0.0, 0.0, exact_yaw_limit);
  EXPECT_FALSE(yaw_boundary.exceeded());
}
TEST(PassageDeviation, InvalidIsVisibleWithoutChangingOriginalComparison) {
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  const auto yaw_invalid = passageDeviation(invalid, 0.0, 0.07, 0.0, 0.05);
  EXPECT_FALSE(yaw_invalid.valid);
  EXPECT_FALSE(yaw_invalid.yaw_exceeded);
  EXPECT_TRUE(yaw_invalid.lateral_exceeded);
  const auto all_invalid = passageDeviation(invalid, 0.0, invalid, 0.0, 0.05);
  EXPECT_FALSE(all_invalid.valid);
  EXPECT_FALSE(all_invalid.exceeded());
}
TEST(PassageDeviation, ManualStallPoseIsWithinSixCentimeterTrialLimit) {
  const auto check = passageDeviation(-0.015703887015818319, 0.0,
      3.4367305884562835, 3.475, 0.0523605);
  EXPECT_TRUE(check.valid);
  EXPECT_DOUBLE_EQ(check.lateral_limit, 0.06);
  EXPECT_FALSE(check.exceeded());
  EXPECT_FALSE(passageDeviation(0.0, 0.0, -0.04, 0.0, 0.0523605).exceeded());
}
TEST(PassageDeviation, ExplicitStricterLimitRemainsSupported) {
  const auto check = passageDeviation(0.0, 0.0, -0.04, 0.0,
                                       0.0523605, 0.035);
  EXPECT_TRUE(check.lateral_exceeded);
  EXPECT_DOUBLE_EQ(check.lateral_limit, 0.035);
}
TEST(PassageDiagnosticJson, NonFiniteNumbersAndUntrustedTextAreSafe) {
  EXPECT_EQ(sentry_tunnel_bt::diagnosticJsonNumber(
                std::numeric_limits<double>::infinity()),
            "null");
  EXPECT_EQ(sentry_tunnel_bt::diagnosticJsonString("\"a\\b\n\t\x01"),
            "\"\\\"a\\\\b\\n\\t\\u0001\"");
}
