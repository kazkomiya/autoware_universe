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

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <string_view>
#include <vector>

namespace
{
using autoware::motion::control::mpc_lateral_controller::ControllerReporter;
using autoware::motion::control::mpc_lateral_controller::MpcLateralController;
using autoware::motion::control::mpc_lateral_controller::MpcLateralControllerConfig;
using autoware::motion::control::mpc_lateral_controller::Repeat;
using autoware::motion::control::mpc_lateral_controller::ReportLevel;
using autoware::motion::control::mpc_lateral_controller::ReportSite;
using autoware::motion::control::trajectory_follower::InputData;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr double wheel_base = 2.74;
constexpr double ctrl_period = 0.03;

/// The values the shipped parameter files carry, given here rather than read from a node.
MpcLateralControllerConfig make_config()
{
  MpcLateralControllerConfig config;
  config.ctrl_period = ctrl_period;

  config.trajectory_filtering.enable_path_smoothing = false;
  config.trajectory_filtering.path_filter_moving_ave_num = 25;
  config.trajectory_filtering.curvature_smoothing_num_traj = 15;
  config.trajectory_filtering.curvature_smoothing_num_ref_steer = 15;
  config.trajectory_filtering.traj_resample_dist = 0.1;
  config.trajectory_filtering.extend_trajectory_for_end_yaw_control = true;

  config.stop_state_entry_ego_speed = 0.001;
  config.stop_state_entry_target_speed = 0.001;
  config.converged_steer_rad = 0.1;
  config.keep_steer_control_until_converged = true;
  config.new_traj_duration_time = 1.0;
  config.new_traj_end_dist = 0.3;
  config.mpc_converged_threshold_rps = 0.01;

  config.ego_nearest_dist_threshold = 3.0;
  config.ego_nearest_yaw_threshold = 1.046;

  config.steering_lpf_cutoff_hz = 3.0;
  config.error_deriv_lpf_cutoff_hz = 5.0;

  config.wheelbase = wheel_base;
  config.steer_lim = 0.70;
  config.vehicle_model_type = "kinematics";
  config.qp_solver_type = "unconstraint_fast";

  config.steer_rate_lim_by_curvature = {{0.0, 1.0}};
  config.steer_rate_lim_by_velocity = {{0.0, 1.0}};

  config.mpc_param.prediction_horizon = 50;
  config.mpc_param.prediction_dt = 0.1;
  config.mpc_param.steer_tau = 0.27;
  config.mpc_param.zero_ff_steer_deg = 0.5;
  config.mpc_param.acceleration_limit = 2.0;
  config.mpc_param.velocity_time_constant = 0.3;
  config.mpc_param.min_prediction_length = 5.0;
  config.mpc_param.nominal_weight.lat_error = 1.0;
  config.mpc_param.nominal_weight.heading_error_squared_vel = 0.3;
  config.mpc_param.nominal_weight.steering_input = 1.0;
  config.mpc_param.nominal_weight.steering_input_squared_vel = 0.25;
  config.mpc_param.nominal_weight.lat_jerk = 0.1;
  config.mpc_param.nominal_weight.steer_acc = 0.000001;
  config.mpc_param.nominal_weight.terminal_lat_error = 1.0;
  config.mpc_param.nominal_weight.terminal_heading_error = 0.1;
  config.mpc_param.low_curvature_weight = config.mpc_param.nominal_weight;

  return config;
}

geometry_msgs::msg::Quaternion make_orientation(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

Trajectory straight_path(const double speed)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  for (double x = 0.0; x <= 20.0; x += 10.0) {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.orientation = make_orientation(0.0);
    point.longitudinal_velocity_mps = static_cast<float>(speed);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

InputData make_input(const Trajectory & path, const double speed)
{
  InputData input;
  input.current_trajectory = path;
  input.current_odometry.header.frame_id = "map";
  input.current_odometry.pose.pose.orientation = make_orientation(0.0);
  input.current_odometry.twist.twist.linear.x = speed;
  input.current_operation_mode.is_autoware_control_enabled = true;
  input.current_operation_mode.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  return input;
}

/// Keeps what the controller reports, so that a test can read it where the node would
/// send it to a logger.
class CollectingReporter : public ControllerReporter
{
public:
  mutable std::vector<std::string> debugs;
  mutable std::vector<std::string> warns;
  mutable std::vector<std::string> errors;

  bool shouldWrite(ReportLevel, Repeat, ReportSite &) const override { return true; }

  void write(const ReportLevel level, std::string_view message) const override
  {
    switch (level) {
      case ReportLevel::debug:
        debugs.emplace_back(message);
        break;
      case ReportLevel::warn:
        warns.emplace_back(message);
        break;
      case ReportLevel::error:
        errors.emplace_back(message);
        break;
      default:
        break;
    }
  }
};

rclcpp::Time at(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}
}  // namespace

/// The controller is built from a configuration alone, with no node to read it from.
TEST(MpcLateralControllerCoreTest, IsNotReadyWithoutTrajectory)
{
  CollectingReporter reporter;
  MpcLateralController controller(make_config(), reporter);

  EXPECT_FALSE(controller.isReady(make_input(Trajectory{}, 1.0)));
}

TEST(MpcLateralControllerCoreTest, IsReadyWithThreePointTrajectory)
{
  CollectingReporter reporter;
  MpcLateralController controller(make_config(), reporter);

  EXPECT_TRUE(controller.isReady(make_input(straight_path(1.0), 1.0)));
}

/// A straight path keeps the command at zero, and the cycle reports nothing to log.
TEST(MpcLateralControllerCoreTest, StraightTrajectoryKeepsSteeringNeutral)
{
  CollectingReporter reporter;
  MpcLateralController controller(make_config(), reporter);
  const auto input = make_input(straight_path(1.0), 1.0);
  controller.isReady(input);

  const auto result = controller.run(input, at(ctrl_period));

  EXPECT_FLOAT_EQ(result.output.control_cmd.steering_tire_angle, 0.0f);
  EXPECT_TRUE(reporter.errors.empty());
}

/// A trajectory of one point is not enough, and the cycle says so instead of logging it.
TEST(MpcLateralControllerCoreTest, ReportsATrajectoryWithTooFewPoints)
{
  CollectingReporter reporter;
  MpcLateralController controller(make_config(), reporter);
  Trajectory one_point;
  one_point.header.frame_id = "map";
  one_point.points.push_back(TrajectoryPoint{});
  const auto input = make_input(one_point, 1.0);

  controller.run(input, at(ctrl_period));

  ASSERT_FALSE(reporter.debugs.empty());
  EXPECT_NE(reporter.debugs.front().find("not enough"), std::string::npos);
}

TEST(MpcLateralControllerCoreTest, ReportsAnUnknownVehicleModel)
{
  CollectingReporter reporter;
  auto config = make_config();
  config.vehicle_model_type = "not_a_model";

  const MpcLateralController controller(config, reporter);

  ASSERT_FALSE(reporter.errors.empty());
  EXPECT_NE(reporter.errors.front().find("vehicle_model_type"), std::string::npos);
}

TEST(MpcLateralControllerCoreTest, IsNotReadyWithAnUnknownVehicleModel)
{
  auto config = make_config();
  config.vehicle_model_type = "not_a_model";
  CollectingReporter reporter;
  MpcLateralController controller(config, reporter);

  EXPECT_FALSE(controller.isReady(make_input(straight_path(1.0), 1.0)));
}
