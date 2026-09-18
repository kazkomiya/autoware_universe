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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_

#include "autoware/mpc_lateral_controller/controller_message.hpp"
#include "autoware/mpc_lateral_controller/lowpass_filter.hpp"
#include "autoware/mpc_lateral_controller/mpc.hpp"
#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware/trajectory_follower_base/control_horizon.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
using autoware_control_msgs::msg::Lateral;
using autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using trajectory_follower::LateralHorizon;

class MpcLateralControllerNode
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit MpcLateralControllerNode(
    rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater);
  virtual ~MpcLateralControllerNode();

  void set_steering_offset(double offset) override { m_steering_offset_ = offset; }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_predicted_traj;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_predicted_traj_frenet;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_resampled_reference_traj;
  rclcpp::Publisher<PoseStamped>::SharedPtr m_pub_nearest_pose;
  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_nearest_segment_traj;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr m_pub_nearest_info;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr m_pub_debug_values;
  rclcpp::Publisher<Float32Stamped>::SharedPtr m_pub_steer_offset;


  std::shared_ptr<diagnostic_updater::Updater>
    diag_updater_{};  // Diagnostic updater for publishing diagnostic data.











  void setStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void setupDiag();




  void setSteeringToHistory(const Lateral & steering);













   * @brief Initialize the timer
   * @param period_s Control period in seconds.
  void initTimer(double period_s);

   * @brief Create the vehicle model based on the provided parameters.
   * @param wheelbase Vehicle's wheelbase.
   * @param steer_lim Steering command limit.
   * @param steer_tau Steering time constant.
   * @param node Reference to the node.
   * @return Pointer to the created vehicle model.
  std::shared_ptr<VehicleModelInterface> createVehicleModel(
    const double wheelbase, const double steer_lim, const double steer_tau, rclcpp::Node & node);

   * @brief Create the quadratic problem solver interface.
   * @param node Reference to the node.
   * @return Pointer to the created QP solver interface.
  std::shared_ptr<QPSolverInterface> createQPSolverInterface(rclcpp::Node & node);

  bool isReady(const trajectory_follower::InputData & input_data) override;

  trajectory_follower::LateralOutput run(
    trajectory_follower::InputData const & input_data) override;





   * @brief Publish the predicted future trajectory.
   * @param predicted_traj Predicted future trajectory to be published.
  void publishPredictedTraj(Trajectory & predicted_traj) const;

   * @brief Publish the MPC debug topic messages (predicted trajectory in Frenet coordinate,
   * resampled reference trajectory, nearest pose, nearest segment trajectory, nearest info).
   * @param debug_msgs MPC debug topic messages to be published.
  void publishDebugMessages(std::optional<MpcDebugTopicMessage> & debug_msgs) const;

   * @brief Publish diagnostic message.
   * @param diagnostic Diagnostic message to be published.
  void publishDebugValues(Float32MultiArrayStamped & diagnostic) const;







  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

   * @brief Declare MPC parameters as ROS parameters to allow tuning on the fly.
   * @param node Reference to the node.
  void declareMPCparameters(rclcpp::Node & node);

   * @brief Callback function called when parameters are changed outside of the node.
   * @param parameters Vector of changed parameters.
   * @return Result of the parameter callback.
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /// Write what the control had to say. Each message stands in its own place in the code,
  /// so each keeps the waiting time of its own message.
  void writeMessages(const std::vector<Message> & messages) const;

  template <typename... Args>
  inline void info_throttle(Args &&... args) const
  {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000, "%s", args...);

  template <typename... Args>
  inline void debug_throttle(Args &&... args) const
  {
    RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 5000, "%s", args...);

  template <typename... Args>
  inline void warn_throttle(Args &&... args) const
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "%s", args...);
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_
