// Copyright 2026 diting
// SPDX-License-Identifier: Apache-2.0
#ifndef SENTRY_TUNNEL_BT__PASSAGE_DIAGNOSTICS_HPP_
#define SENTRY_TUNNEL_BT__PASSAGE_DIAGNOSTICS_HPP_
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace sentry_tunnel_bt {
// Preserve the existing strict > comparisons. valid is diagnostic only.
struct PassageDeviationCheck {
  double actual_yaw, target_yaw, yaw_error, yaw_limit;
  double body_y, center_y, signed_lateral_error, lateral_limit;
  bool valid, yaw_exceeded, lateral_exceeded;
  bool exceeded() const { return yaw_exceeded || lateral_exceeded; }
};
inline PassageDeviationCheck passageDeviation(double actual_yaw,
                                              double target_yaw, double body_y,
                                              double center_y, double yaw_limit,
                                              double lateral_limit = 0.06) {
  const double yaw_error = std::abs(std::atan2(
      std::sin(target_yaw - actual_yaw), std::cos(target_yaw - actual_yaw)));
  const double lateral_error = body_y - center_y;
  const bool valid = std::isfinite(actual_yaw) && std::isfinite(target_yaw) &&
                     std::isfinite(body_y) && std::isfinite(center_y) &&
                     std::isfinite(yaw_limit) && std::isfinite(lateral_limit) &&
                     yaw_limit > 0.0 && lateral_limit > 0.0;
  return {actual_yaw,
          target_yaw,
          yaw_error,
          yaw_limit,
          body_y,
          center_y,
          lateral_error,
          lateral_limit,
          valid,
          yaw_error > yaw_limit,
          std::abs(lateral_error) > lateral_limit};
}
inline std::string diagnosticJsonNumber(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << value;
  return out.str();
}
inline std::string diagnosticJsonString(const std::string &value) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value) {
    switch (c) {
    case '"':
      out << "\\\"";
      break;
    case '\\':
      out << "\\\\";
      break;
    case '\b':
      out << "\\b";
      break;
    case '\f':
      out << "\\f";
      break;
    case '\n':
      out << "\\n";
      break;
    case '\r':
      out << "\\r";
      break;
    case '\t':
      out << "\\t";
      break;
    default:
      if (c < 0x20) {
        out << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<int>(c) << std::dec;
      } else {
        out << c;
      }
    }
  }
  out << '"';
  return out.str();
}
} // namespace sentry_tunnel_bt
#endif // SENTRY_TUNNEL_BT__PASSAGE_DIAGNOSTICS_HPP_
