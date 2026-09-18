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

#include <string>

namespace autoware::motion::control::mpc_lateral_controller
{
/// What the control met. The node holds the wording, the severity and how long the same
/// message waits before it is written again, so each identifier stands for one message.
enum class MessageId {
  spline_resample_failed,
  path_filter_failed,
  resampled_trajectory_empty,
  state_vehicle_model_undefined,
  delay_compensation_resample_failed,
  qp_solver_failed,
  qp_solver_warning,
  trajectory_size_inconsistent,
  world_coordinate_prediction_unsupported,
};

struct Message
{
  MessageId id;
  /// Text the control received from elsewhere. Empty when the node holds the wording.
  std::string detail;
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__CONTROLLER_MESSAGE_HPP_
