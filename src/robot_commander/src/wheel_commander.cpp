#include <robot_commander/wheel_commander.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace robot_commander
{

// ============================================================================
//  Constructor
// ============================================================================

WheelCommander::WheelCommander(
  rclcpp::Node::SharedPtr node,
  const std::string & command_topic)
: node_(std::move(node)),
  command_topic_(command_topic)
{
  // Create the velocity command publisher
  velocity_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
    command_topic_, rclcpp::SystemDefaultsQoS());

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

  // ------------------------------------------------------------------
  // Publish velocity command
  // ------------------------------------------------------------------
  std_msgs::msg::Float64MultiArray msg;
  msg.data = velocities;

  RCLCPP_INFO(node_->get_logger(),
              "[WheelCommander] Publishing velocity command [%.3f, %.3f, %.3f, %.3f] "
              "to %s for %.1f s",
              velocities[0], velocities[1], velocities[2], velocities[3],
              command_topic_.c_str(), duration);

  velocity_pub_->publish(msg);

  // ------------------------------------------------------------------
  // Wait for the specified duration using simulation time
  // ------------------------------------------------------------------
  const auto deadline = node_->now() + rclcpp::Duration::from_seconds(duration);
  while (rclcpp::ok() && node_->now() < deadline) {
    rclcpp::spin_some(node_);
  }

  // ------------------------------------------------------------------
  // Stop (publish zero velocities)
  // ------------------------------------------------------------------
  std::fill(msg.data.begin(), msg.data.end(), 0.0);
  velocity_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[WheelCommander] Stopped");

  return rclcpp::ok();
}

}  // namespace robot_commander
