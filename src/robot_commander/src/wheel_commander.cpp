#include <robot_commander/wheel_commander.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_commander
{

// ============================================================================
//  Constructor
// ============================================================================

WheelCommander::WheelCommander(
  rclcpp::Node::SharedPtr node,
  const std::string & action_topic)
: node_(std::move(node)),
  action_topic_(action_topic)
{
  // Create the action client (lazy: will connect on first call)
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(node_, action_topic_);

  // Subscribe to /joint_states to keep current positions up to date
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        current_positions_[msg->name[i]] = msg->position[i];
      }
    });
}

// ============================================================================
//  waitForJointStates
// ============================================================================

bool WheelCommander::waitForJointStates(const std::chrono::seconds & timeout)
{
  RCLCPP_INFO(node_->get_logger(), "[WheelCommander] Waiting for /joint_states ...");
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    if (current_positions_.count("wheel_joint_1") &&
        current_positions_.count("wheel_joint_2") &&
        current_positions_.count("wheel_joint_3") &&
        current_positions_.count("wheel_joint_4")) {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[WheelCommander] Timeout waiting for wheel joint states");
      return false;
    }
  }
  return false;
}

// ============================================================================
//  waitForActionServer
// ============================================================================

bool WheelCommander::waitForActionServer(const std::chrono::seconds & timeout)
{
  RCLCPP_INFO(node_->get_logger(),
              "[WheelCommander] Waiting for action server %s ...",
              action_topic_.c_str());
  if (!action_client_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] Action server not available: %s",
                 action_topic_.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "[WheelCommander] Connected to %s", action_topic_.c_str());
  return true;
}

// ============================================================================
//  driveTurn
// ============================================================================

bool WheelCommander::driveTurn(double linear_speed, double duration)
{
  const double angular_vel = linear_speed / WHEEL_RADIUS;

  RCLCPP_INFO(node_->get_logger(),
              "[WheelCommander] driveTurn: all wheels @ %.3f rad/s (%.3f m/s) for %.1f s",
              angular_vel, linear_speed, duration);

  const std::vector<double> velocities = {
    angular_vel,   // wheel_joint_4
    angular_vel,   // wheel_joint_3
    angular_vel,   // wheel_joint_2
    angular_vel    // wheel_joint_1
  };
  return driveWithVelocities(velocities, duration);
}

// ============================================================================
//  driveForward
// ============================================================================

bool WheelCommander::driveForward(double linear_speed, double duration)
{
  const double angular_vel = linear_speed / WHEEL_RADIUS;

  RCLCPP_INFO(node_->get_logger(),
              "[WheelCommander] driveForward: j1,j2 @ +%.3f rad/s, j3,j4 @ -%.3f rad/s  "
              "(%.3f m/s) for %.1f s",
              angular_vel, angular_vel, linear_speed, duration);

  const std::vector<double> velocities = {
    -angular_vel,   // wheel_joint_4 (reverse)
    -angular_vel,   // wheel_joint_3 (reverse)
    angular_vel,    // wheel_joint_2 (forward)
    angular_vel     // wheel_joint_1 (forward)
  };
  return driveWithVelocities(velocities, duration);
}

// ============================================================================
//  driveWithVelocities  (core implementation)
// ============================================================================

bool WheelCommander::driveWithVelocities(const std::vector<double> & velocities, double duration)
{
  if (!rclcpp::ok()) {
    return false;
  }

  // Ensure we have joint state data
  if (!current_positions_.count("wheel_joint_1")) {
    RCLCPP_WARN(node_->get_logger(),
                "[WheelCommander] No joint state data yet — call waitForJointStates() first");
    return false;
  }

  // ------------------------------------------------------------------
  // Build trajectory goal
  // ------------------------------------------------------------------
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = jointNames();

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(4);
  point.velocities.resize(4);

  for (int i = 0; i < 4; ++i) {
    const auto & jn = jointNames()[i];
    point.positions[i] = current_positions_[jn] + velocities[i] * duration;
    point.velocities[i] = velocities[i];
  }
  point.time_from_start = rclcpp::Duration::from_seconds(duration);
  goal_msg.trajectory.points.push_back(std::move(point));

  // ------------------------------------------------------------------
  // Send goal
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "[WheelCommander] Sending trajectory goal ...");

  auto send_goal_future = action_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, send_goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[WheelCommander] Failed to send goal");
    return false;
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "[WheelCommander] Goal was rejected by controller");
    return false;
  }

  // ------------------------------------------------------------------
  // Wait for result
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "[WheelCommander] Goal accepted, waiting for execution ...");

  auto result_future = action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[WheelCommander] Interrupted while waiting for result");
    return false;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "[WheelCommander] Trajectory execution succeeded!");
    return true;
  }

  RCLCPP_WARN(node_->get_logger(),
              "[WheelCommander] Trajectory execution failed (code %d)",
              static_cast<int>(result.code));
  return false;
}

}  // namespace robot_commander
