// Copyright 2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc_lateral_controller_node.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

void MpcLateralControllerNode::writeMessages(const std::vector<Message> & messages) const
{
  for (const auto & message : messages) {
    switch (message.id) {
      case MessageId::spline_resample_failed:
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000, "%s", message.text.c_str());
        break;
      case MessageId::qp_solver_warning:
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "%s", message.text.c_str());
        break;
      case MessageId::qp_solver_failed:
        RCLCPP_WARN(logger_, "%s", message.text.c_str());
        break;
      case MessageId::path_filter_failed:
        RCLCPP_DEBUG(logger_, "%s", message.text.c_str());
        break;
      case MessageId::state_vehicle_model_undefined:
      case MessageId::delay_compensation_resample_failed:
      case MessageId::trajectory_size_inconsistent:
      case MessageId::world_coordinate_prediction_unsupported:
        RCLCPP_ERROR(logger_, "%s", message.text.c_str());
        break;
    }
  }
}
void MpcLateralControllerNode::setStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (m_mpc_solved_status.result) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "MPC succeeded.");
  } else {
    const std::string error_msg = "MPC failed due to " + m_mpc_solved_status.reason;
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_msg);
  }
}
void MpcLateralControllerNode::setupDiag()
{
  diag_updater_->add("MPC_solve_checker", [&](auto & stat) { setStatus(stat); });
}
void MpcLateralControllerNode::publishPredictedTraj(Trajectory & predicted_traj) const
{
  m_pub_predicted_traj->publish(predicted_traj);
}
void MpcLateralControllerNode::publishDebugMessages(
  std::optional<MpcDebugTopicMessage> & debug_msgs) const
{
  if (!debug_msgs) {
    return;
  }

  m_pub_predicted_traj_frenet->publish(debug_msgs->predicted_trajectory_frenet);
  m_pub_resampled_reference_traj->publish(debug_msgs->resampled_reference_trajectory);
  m_pub_nearest_pose->publish(debug_msgs->nearest_pose);
  m_pub_nearest_segment_traj->publish(debug_msgs->nearest_segment_trajectory);
  m_pub_nearest_info->publish(debug_msgs->nearest_info);
}
void MpcLateralControllerNode::publishDebugValues(Float32MultiArrayStamped & debug_values) const
{
  m_pub_debug_values->publish(debug_values);

  Float32Stamped offset;
  offset.stamp = clock_->now();
  offset.data = static_cast<float>(m_steering_offset_filtered_);
  m_pub_steer_offset->publish(offset);
}
void MpcLateralControllerNode::declareMPCparameters(rclcpp::Node & node)
{
  m_mpc->m_param.prediction_horizon = node.declare_parameter<int>("mpc_prediction_horizon");
  m_mpc->m_param.prediction_dt = node.declare_parameter<double>("mpc_prediction_dt");

  const auto dp = [&](const auto & param) { return node.declare_parameter<double>(param); };

  auto & nw = m_mpc->m_param.nominal_weight;
  nw.lat_error = dp("mpc_weight_lat_error");
  nw.heading_error = dp("mpc_weight_heading_error");
  nw.heading_error_squared_vel = dp("mpc_weight_heading_error_squared_vel");
  nw.steering_input = dp("mpc_weight_steering_input");
  nw.steering_input_squared_vel = dp("mpc_weight_steering_input_squared_vel");
  nw.lat_jerk = dp("mpc_weight_lat_jerk");
  nw.steer_rate = dp("mpc_weight_steer_rate");
  nw.steer_acc = dp("mpc_weight_steer_acc");
  nw.terminal_lat_error = dp("mpc_weight_terminal_lat_error");
  nw.terminal_heading_error = dp("mpc_weight_terminal_heading_error");

  auto & lcw = m_mpc->m_param.low_curvature_weight;
  lcw.lat_error = dp("mpc_low_curvature_weight_lat_error");
  lcw.heading_error = dp("mpc_low_curvature_weight_heading_error");
  lcw.heading_error_squared_vel = dp("mpc_low_curvature_weight_heading_error_squared_vel");
  lcw.steering_input = dp("mpc_low_curvature_weight_steering_input");
  lcw.steering_input_squared_vel = dp("mpc_low_curvature_weight_steering_input_squared_vel");
  lcw.lat_jerk = dp("mpc_low_curvature_weight_lat_jerk");
  lcw.steer_rate = dp("mpc_low_curvature_weight_steer_rate");
  lcw.steer_acc = dp("mpc_low_curvature_weight_steer_acc");
  m_mpc->m_param.low_curvature_thresh_curvature = dp("mpc_low_curvature_thresh_curvature");

  m_mpc->m_param.zero_ff_steer_deg = dp("mpc_zero_ff_steer_deg");
  m_mpc->m_param.acceleration_limit = dp("mpc_acceleration_limit");
  m_mpc->m_param.velocity_time_constant = dp("mpc_velocity_time_constant");
  m_mpc->m_param.min_prediction_length = dp("mpc_min_prediction_length");
}
rcl_interfaces::msg::SetParametersResult MpcLateralControllerNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  MPCParam param = m_mpc->m_param;

  using MPCUtils::update_param;
  try {
    auto & nw = param.nominal_weight;
    auto & lcw = param.low_curvature_weight;

    update_param(parameters, "mpc_prediction_horizon", param.prediction_horizon);
    update_param(parameters, "mpc_prediction_dt", param.prediction_dt);

    const std::string ns_nw = "mpc_weight_";
    update_param(parameters, ns_nw + "lat_error", nw.lat_error);
    update_param(parameters, ns_nw + "heading_error", nw.heading_error);
    update_param(parameters, ns_nw + "heading_error_squared_vel", nw.heading_error_squared_vel);
    update_param(parameters, ns_nw + "steering_input", nw.steering_input);
    update_param(parameters, ns_nw + "steering_input_squared_vel", nw.steering_input_squared_vel);
    update_param(parameters, ns_nw + "lat_jerk", nw.lat_jerk);
    update_param(parameters, ns_nw + "steer_rate", nw.steer_rate);
    update_param(parameters, ns_nw + "steer_acc", nw.steer_acc);
    update_param(parameters, ns_nw + "terminal_lat_error", nw.terminal_lat_error);
    update_param(parameters, ns_nw + "terminal_heading_error", nw.terminal_heading_error);

    const std::string ns_lcw = "mpc_low_curvature_weight_";
    update_param(parameters, ns_lcw + "lat_error", lcw.lat_error);
    update_param(parameters, ns_lcw + "heading_error", lcw.heading_error);
    update_param(parameters, ns_lcw + "heading_error_squared_vel", lcw.heading_error_squared_vel);
    update_param(parameters, ns_lcw + "steering_input", lcw.steering_input);
    update_param(parameters, ns_lcw + "steering_input_squared_vel", lcw.steering_input_squared_vel);
    update_param(parameters, ns_lcw + "lat_jerk", lcw.lat_jerk);
    update_param(parameters, ns_lcw + "steer_rate", lcw.steer_rate);
    update_param(parameters, ns_lcw + "steer_acc", lcw.steer_acc);

    update_param(
      parameters, "mpc_low_curvature_thresh_curvature", param.low_curvature_thresh_curvature);

    update_param(parameters, "mpc_zero_ff_steer_deg", param.zero_ff_steer_deg);
    update_param(parameters, "mpc_acceleration_limit", param.acceleration_limit);
    update_param(parameters, "mpc_velocity_time_constant", param.velocity_time_constant);
    update_param(parameters, "mpc_min_prediction_length", param.min_prediction_length);

    // initialize input buffer
    update_param(parameters, "input_delay", param.input_delay);
    const double delay_step = std::round(param.input_delay / m_mpc->m_ctrl_period);
    const double delay = delay_step * m_mpc->m_ctrl_period;
    if (param.input_delay != delay) {
      param.input_delay = delay;
      m_mpc->m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
    }

    // transaction succeeds, now assign values
    m_mpc->m_param = param;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
