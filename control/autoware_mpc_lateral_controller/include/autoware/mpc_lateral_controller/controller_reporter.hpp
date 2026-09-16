// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_REPORTER_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_REPORTER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <fmt/format.h>

#include <string_view>
#include <utility>

namespace autoware::motion::control::mpc_lateral_controller
{
enum class ReportLevel { debug, info, warn, error };

/// How long the same report waits before it is written again.
struct Repeat
{
  double after_s{0.0};
};

/// Writes the report on every call.
inline constexpr Repeat every_time{0.0};

/// State of one place in the code where a report is written. Each place keeps its own, so
/// that one report does not delay another. The starting value is zero, as the logging
/// macros of rclcpp use for the same purpose.
struct ReportSite
{
  double last_sent_s{0.0};
};

/// Where the control reports what it meets while it runs. The node side writes the reports
/// to the logger of the node; a test can read them instead.
///
/// Use the macros below rather than these two functions. They build the message only when
/// it would be read, and they give each place in the code its own ReportSite.
class ControllerReporter
{
public:
  virtual ~ControllerReporter() = default;

  /// Whether a report of this level, written at this place and at this moment, would be read.
  virtual bool shouldWrite(ReportLevel level, Repeat repeat, ReportSite & site) const = 0;

  /// Write the finished message.
  virtual void write(ReportLevel level, std::string_view message) const = 0;
};

/// A reporter that drops everything, for a caller that wants no report at all.
class NullReporter : public ControllerReporter
{
public:
  bool shouldWrite(ReportLevel, Repeat, ReportSite &) const override { return false; }
  void write(ReportLevel, std::string_view) const override {}
};

/// Writes what the control reports to the logger of a node.
class LoggingReporter : public ControllerReporter
{
public:
  LoggingReporter(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  : logger_(std::move(logger)), clock_(std::move(clock))
  {
  }

  bool shouldWrite(const ReportLevel level, const Repeat repeat, ReportSite & site) const override
  {
    if (!rcutils_logging_logger_is_enabled_for(logger_.get_name(), severityOf(level))) {
      return false;
    }
    if (repeat.after_s <= 0.0) {
      return true;
    }

    const double now = clock_->now().seconds();
    if (now < site.last_sent_s + repeat.after_s) {
      return false;
    }
    site.last_sent_s = now;
    return true;
  }

  void write(const ReportLevel level, std::string_view message) const override
  {
    const auto size = static_cast<int>(message.size());
    switch (level) {
      case ReportLevel::debug:
        RCLCPP_DEBUG(logger_, "%.*s", size, message.data());
        break;
      case ReportLevel::info:
        RCLCPP_INFO(logger_, "%.*s", size, message.data());
        break;
      case ReportLevel::warn:
        RCLCPP_WARN(logger_, "%.*s", size, message.data());
        break;
      case ReportLevel::error:
        RCLCPP_ERROR(logger_, "%.*s", size, message.data());
        break;
    }
  }

private:
  static int severityOf(const ReportLevel level)
  {
    switch (level) {
      case ReportLevel::debug:
        return RCUTILS_LOG_SEVERITY_DEBUG;
      case ReportLevel::info:
        return RCUTILS_LOG_SEVERITY_INFO;
      case ReportLevel::warn:
        return RCUTILS_LOG_SEVERITY_WARN;
      case ReportLevel::error:
        return RCUTILS_LOG_SEVERITY_ERROR;
    }
    return RCUTILS_LOG_SEVERITY_DEBUG;
  }

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace autoware::motion::control::mpc_lateral_controller

/// Write a report through `reporter`, at most once per `repeat`. The message is built only
/// when it would be read, so the arguments are not evaluated otherwise. The waiting time is
/// kept for this place in the code alone.
#define AW_REPORT(reporter, level, repeat, ...)                                 \
  do {                                                                              \
    static ::autoware::motion::control::mpc_lateral_controller::ReportSite aw_site; \
    if ((reporter)->shouldWrite((level), (repeat), aw_site)) {                      \
      (reporter)->write((level), ::fmt::format(__VA_ARGS__));                       \
    }                                                                               \
  } while (0)

#define AW_LEVEL(name) ::autoware::motion::control::mpc_lateral_controller::ReportLevel::name
#define AW_AFTER(seconds)                                 \
  ::autoware::motion::control::mpc_lateral_controller::Repeat \
  {                                                           \
    (seconds)                                                 \
  }

#define AW_DEBUG(reporter, ...) \
  AW_REPORT(                    \
    reporter, AW_LEVEL(debug),  \
    ::autoware::motion::control::mpc_lateral_controller::every_time, __VA_ARGS__)
#define AW_INFO(reporter, ...)                                                                 \
  AW_REPORT(                                                                                   \
    reporter, AW_LEVEL(info), ::autoware::motion::control::mpc_lateral_controller::every_time, \
    __VA_ARGS__)
#define AW_WARN(reporter, ...)                                                                 \
  AW_REPORT(                                                                                   \
    reporter, AW_LEVEL(warn), ::autoware::motion::control::mpc_lateral_controller::every_time, \
    __VA_ARGS__)
#define AW_ERROR(reporter, ...) \
  AW_REPORT(                    \
    reporter, AW_LEVEL(error),  \
    ::autoware::motion::control::mpc_lateral_controller::every_time, __VA_ARGS__)

#define AW_DEBUG_THROTTLE(reporter, seconds, ...) \
  AW_REPORT(reporter, AW_LEVEL(debug), AW_AFTER(seconds), __VA_ARGS__)
#define AW_INFO_THROTTLE(reporter, seconds, ...) \
  AW_REPORT(reporter, AW_LEVEL(info), AW_AFTER(seconds), __VA_ARGS__)
#define AW_WARN_THROTTLE(reporter, seconds, ...) \
  AW_REPORT(reporter, AW_LEVEL(warn), AW_AFTER(seconds), __VA_ARGS__)
#define AW_ERROR_THROTTLE(reporter, seconds, ...) \
  AW_REPORT(reporter, AW_LEVEL(error), AW_AFTER(seconds), __VA_ARGS__)

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_REPORTER_HPP_
