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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_EVENT_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_EVENT_HPP_

#include <iterator>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
/// Something the control met while it ran. The node turns it into a message; the control
/// itself holds no text and no logger.
enum class EventId {
  spline_resample_failed,
  path_filter_failed,
  resampled_trajectory_empty,
  state_vehicle_model_undefined,
  delay_compensation_resample_failed,
  qp_solver_failed,
  qp_solver_warning,
  vehicle_model_type_undefined,
  qp_solver_type_undefined,
  mpc_failed,
  stopped_state_detected,
  mpc_not_solved,
  trajectory_shape_changed,
  no_vehicle_model,
  no_qp_solver,
  reference_trajectory_empty,
  trajectory_too_few_points,
  trajectory_invalid,
  steering_not_converged,
  trajectory_size_inconsistent,
  nearest_pose_trajectory_empty,
  nearest_pose_interp_failed,
  temporal_nearest_pose_failed,
  trajectory_too_short,
  world_coordinate_prediction_unsupported,
};

struct Event
{
  /// Decides the severity and the waiting time the node applies, and keeps one message
  /// apart from another.
  EventId id;
  /// The message itself, written where the event was met.
  std::string text;
};

using Events = std::vector<Event>;

/// A value the control computed, together with what it met on the way.
template <typename T>
struct WithEvents
{
  T value;
  Events events;
};

/// The result of a call that can fail, together with what it met on the way.
template <typename T>
struct Outcome
{
  /// Empty when the call failed.
  std::optional<T> value;
  /// Why the call failed. Empty when it did not.
  std::string reason;
  Events events;
};

inline void report(Events & events, const EventId id, std::string text)
{
  events.push_back(Event{id, std::move(text)});
}

/// Add what a call that is now finished met to what the caller has met so far.
inline void append(Events & to, Events && from)
{
  to.insert(to.end(), std::make_move_iterator(from.begin()), std::make_move_iterator(from.end()));
}
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_EVENT_HPP_
