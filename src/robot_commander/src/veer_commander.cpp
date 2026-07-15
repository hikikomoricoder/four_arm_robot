#include <robot_commander/veer_commander.hpp>

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

VeerCommander::VeerCommander(
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

bool VeerCommander::waitForJointStates(const std::chrono::seconds & timeout)
{
  RCLCPP_INFO(node_->get_logger(), "[VeerCommander] Waiting for /joint_states ...");
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    if (current_positions_.count("arm_veer_joint_1") &&
        current_positions_.count("arm_veer_joint_2") &&
        current_positions_.count("arm_veer_joint_3") &&
        current_positions_.count("arm_veer_joint_4")) {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[VeerCommander] Timeout waiting for veer joint states");
      return false;
    }
  }
  return false;
}

// ============================================================================
//  waitForActionServer
// ============================================================================

bool VeerCommander::waitForActionServer(const std::chrono::seconds & timeout)
{
  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] Waiting for action server %s ...",
              action_topic_.c_str());
  if (!action_client_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] Action server not available: %s",
                 action_topic_.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "[VeerCommander] Connected to %s", action_topic_.c_str());
  return true;
}

// ============================================================================
//  goHome
// ============================================================================

bool VeerCommander::goHome(double duration)
{
  const auto & home = homePositions();

  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] goHome: sending all veer joints to "
              "[%.3f, %.3f, %.3f, %.3f] rad over %.1f s",
              home[0], home[1], home[2], home[3], duration);

  return sendPositionGoal(home, duration);
}

// ============================================================================
//  setForwardState
// ============================================================================

bool VeerCommander::setForwardState(double duration)
{
  // Ensure we have joint state data
  if (!current_positions_.count("arm_veer_joint_1")) {
    RCLCPP_WARN(node_->get_logger(),
                "[VeerCommander] No joint state data yet — call waitForJointStates() first");
    return false;
  }

  // Build target positions relative to the current state:
  //   j1 stays, j2 -= pi/2, j3 stays, j4 -= pi/2
  // Controller joint order: [arm_veer_joint_4, arm_veer_joint_3,
  //                          arm_veer_joint_2, arm_veer_joint_1]
  const auto & names = jointNames();
  std::vector<double> targets(4);
  for (size_t i = 0; i < 4; ++i) {
    targets[i] = current_positions_[names[i]];
  }
  // Index mapping: names[0]=j4, names[1]=j3, names[2]=j2, names[3]=j1
  targets[0] -= M_PI_2;  // arm_veer_joint_4  -90°
  // arm_veer_joint_3 stays (no change)
  targets[2] -= M_PI_2;  // arm_veer_joint_2  -90°
  // arm_veer_joint_1 stays (no change)

  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setForwardState: "
              "j1 stays (%.3f), j2 -pi/2 -> %.3f, "
              "j3 stays (%.3f), j4 -pi/2 -> %.3f  over %.1f s",
              targets[3], targets[2], targets[1], targets[0], duration);

  return sendPositionGoal(targets, duration);
}

// ============================================================================
//  sendPositionGoal  (core implementation)
// ============================================================================

bool VeerCommander::sendPositionGoal(const std::vector<double> & positions, double duration)
{
  if (!rclcpp::ok()) {
    return false;
  }

  // Ensure we have joint state data
  if (!current_positions_.count("arm_veer_joint_1")) {
    RCLCPP_WARN(node_->get_logger(),
                "[VeerCommander] No joint state data yet — call waitForJointStates() first");
    return false;
  }

  // ------------------------------------------------------------------
  // Build trajectory goal
  // ------------------------------------------------------------------
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = jointNames();
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(duration + 3.0);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(4);
  point.velocities.resize(4);

  for (size_t i = 0; i < 4; ++i) {
    const auto & jn = jointNames()[i];
    point.positions[i] = positions[i];
    // Compute velocity so the joint arrives at the target in `duration`
    point.velocities[i] =
      (positions[i] - current_positions_[jn]) / duration;
  }
  point.time_from_start = rclcpp::Duration::from_seconds(duration);
  goal_msg.trajectory.points.push_back(std::move(point));

  // ------------------------------------------------------------------
  // Send goal
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "[VeerCommander] Sending trajectory goal ...");

  auto send_goal_future = action_client_->async_send_goal(goal_msg);
  if (rclcpp::spin_until_future_complete(node_, send_goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[VeerCommander] Failed to send goal");
    return false;
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "[VeerCommander] Goal was rejected by controller");
    return false;
  }

  // ------------------------------------------------------------------
  // Wait for result
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "[VeerCommander] Goal accepted, waiting for execution ...");

  auto result_future = action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[VeerCommander] Interrupted while waiting for result");
    return false;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "[VeerCommander] Trajectory execution succeeded!");
    return true;
  }

  RCLCPP_WARN(node_->get_logger(),
              "[VeerCommander] Trajectory execution failed (code %d)",
              static_cast<int>(result.code));
  return false;
}

}  // namespace robot_commander
