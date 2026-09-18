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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_MESSAGE_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_MESSAGE_HPP_

namespace autoware::motion::control::mpc_lateral_controller
{
/// What the control has to say. The identifier keeps the waiting time of one message
/// apart from that of another.
enum class MessageId {
  spline_resample_failed,
  path_filter_failed,
  state_vehicle_model_undefined,
  delay_compensation_resample_failed,
  qp_solver_failed,
  qp_solver_warning,
  trajectory_size_inconsistent,
  world_coordinate_prediction_unsupported,
  vehicle_model_type_undefined,
  qp_solver_type_undefined,
  mpc_failed,
  stopped_state_detected,
  mpc_not_solved,
  trajectory_shape_changed,
  vehicle_model_missing,
  qp_solver_missing,
  trajectory_empty,
  trajectory_too_short,
  trajectory_invalid,
  steering_not_converged,

  /// The number of the identifiers above, used to size a table.
  count,
};

}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_MESSAGE_HPP_
