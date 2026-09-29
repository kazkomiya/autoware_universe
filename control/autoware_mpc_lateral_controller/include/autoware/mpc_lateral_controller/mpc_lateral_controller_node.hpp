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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
using autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::PoseStamped;

/// The node side of the lateral controller. It declares the parameters, builds the
/// controller from them, publishes what a cycle produced and writes what it reported.
class MpcLateralControllerNode : public trajectory_follower::LateralControllerBase
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

  /// Writes what the control says to the logger of the node. A message waits as long as
  /// the call that made it asked for; the wait is kept for each MessageId of its own.
  class LoggingReporter : public ControllerReporter
  {
  public:
    LoggingReporter(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
    : logger_(std::move(logger)), clock_(std::move(clock))
    {
    }

  protected:
    void report(Level level, MessageId id, Repeat repeat, std::string text) const override;

  private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    /// When each message was written last, in seconds of the clock of the node. Zero
    /// stands for a message not written yet, as the logging macros of rclcpp use.
    mutable std::array<double, static_cast<size_t>(MessageId::count)> last_sent_s_{};
  };

  LoggingReporter reporter_;

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

  /**
   * @brief Check if all necessary data is received and ready to run the control.
   * @param input_data Input data required for control calculation.
   * @return True if the data is ready, false otherwise.
   */
  bool isReady(const trajectory_follower::InputData & input_data) override;

  /**
   * @brief Compute the control command for path following with a constant control period.
   * @param input_data Input data required for control calculation.
   * @return Lateral output control command.
   */
  trajectory_follower::LateralOutput run(
    trajectory_follower::InputData const & input_data) override;

  /**
   * @brief Publish the predicted future trajectory.
   * @param predicted_traj Predicted future trajectory to be published.
   */
  void publishPredictedTraj(Trajectory & predicted_traj) const;

  /**
   * @brief Publish the MPC debug topic messages (predicted trajectory in Frenet coordinate,
   * resampled reference trajectory, nearest pose, nearest segment trajectory, nearest info).
   * @param debug_msgs MPC debug topic messages to be published.
   */
  void publishDebugMessages(std::optional<MpcDebugTopicMessage> & debug_msgs) const;

  /**
   * @brief Publish diagnostic message.
   * @param diagnostic Diagnostic message to be published.
   */
  void publishDebugValues(Float32MultiArrayStamped & diagnostic) const;


  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  /**
   * @brief Declare MPC parameters as ROS parameters to allow tuning on the fly.
   * @param node Reference to the node.
   */
  void declareMPCparameters(rclcpp::Node & node);

  /**
   * @brief Callback function called when parameters are changed outside of the node.
   * @param parameters Vector of changed parameters.
   * @return Result of the parameter callback.
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_NODE_HPP_
