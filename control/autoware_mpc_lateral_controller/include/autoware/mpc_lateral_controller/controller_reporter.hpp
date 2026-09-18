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

#include "autoware/mpc_lateral_controller/controller_message.hpp"

#include <string>
#include <utility>

namespace autoware::motion::control::mpc_lateral_controller
{
enum class Level { debug, info, warn, error };

/// How long the same message waits before it is written again.
struct Repeat
{
  double after_s{0.0};
};

/// Writes the message on every call.
inline constexpr Repeat every_time{0.0};

/// Where the control says what it meets. The node writes it to the logger of the node; a
/// test reads it instead. The waiting time is kept for each MessageId of its own, so one
/// message never delays another.
class ControllerReporter
{
public:
  virtual ~ControllerReporter() = default;

  void debug(const MessageId id, std::string text) const
  {
    report(Level::debug, id, every_time, std::move(text));
  }
  void info(const MessageId id, std::string text) const
  {
    report(Level::info, id, every_time, std::move(text));
  }
  void warn(const MessageId id, std::string text) const
  {
    report(Level::warn, id, every_time, std::move(text));
  }
  void error(const MessageId id, std::string text) const
  {
    report(Level::error, id, every_time, std::move(text));
  }

  void debug(const MessageId id, const Repeat repeat, std::string text) const
  {
    report(Level::debug, id, repeat, std::move(text));
  }
  void info(const MessageId id, const Repeat repeat, std::string text) const
  {
    report(Level::info, id, repeat, std::move(text));
  }
  void warn(const MessageId id, const Repeat repeat, std::string text) const
  {
    report(Level::warn, id, repeat, std::move(text));
  }
  void error(const MessageId id, const Repeat repeat, std::string text) const
  {
    report(Level::error, id, repeat, std::move(text));
  }

protected:
  virtual void report(Level level, MessageId id, Repeat repeat, std::string text) const = 0;
};

/// A reporter that drops everything, for a caller that wants no message at all.
class NullReporter : public ControllerReporter
{
protected:
  void report(Level, MessageId, Repeat, std::string) const override {}
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_REPORTER_HPP_
