#include <robot_commander/arm_commander.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <chrono>
#include <string>
#include <vector>

namespace robot_commander
{

namespace
{
// URDF position limits (rad):
//   arm_joint_1_x: [-3.14, 3.14]
//   arm_joint_3/5/7_x: [-2.35, 2.35]
// Indexed by the position inside each per-arm block [j1, j3, j5, j7].
constexpr double kPositionLimits[4][2] = {
  {-3.14, 3.14},
  {-2.35, 2.35},
  {-2.35, 2.35},
  {-2.35, 2.35}};
}  // namespace

// ============================================================================
//  Constructor
// ============================================================================

ArmCommander::ArmCommander(
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

bool ArmCommander::waitForJointStates(const std::chrono::seconds & timeout)
{
  RCLCPP_INFO(node_->get_logger(), "[ArmCommander] Waiting for /joint_states ...");
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node_);
    bool all_received = true;
    for (const auto & jn : jointNames()) {
      if (!current_positions_.count(jn)) {
        all_received = false;
        break;
      }
    }
    if (all_received) {
      return true;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[ArmCommander] Timeout waiting for arm joint states");
      return false;
    }
  }
  return false;
}

// ============================================================================
//  waitForActionServer
// ============================================================================

bool ArmCommander::waitForActionServer(const std::chrono::seconds & timeout)
{
  RCLCPP_INFO(node_->get_logger(),
              "[ArmCommander] Waiting for action server %s ...",
              action_topic_.c_str());
  if (!action_client_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ArmCommander] Action server not available: %s",
                 action_topic_.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "[ArmCommander] Connected to %s", action_topic_.c_str());
  return true;
}

// ============================================================================
//  asyncSendPositionGoal  (non-blocking goal send)
// ============================================================================

bool ArmCommander::asyncSendPositionGoal(
  const std::vector<double> & positions, double duration,
  SendGoalFuture & goal_future)
{
  if (!rclcpp::ok()) {
    return false;
  }
  if (positions.size() != jointNames().size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ArmCommander] asyncSendPositionGoal: expected %zu positions, got %zu",
                 jointNames().size(), positions.size());
    return false;
  }

  // Ensure we have joint state data
  if (!current_positions_.count("arm_joint_1_1")) {
    RCLCPP_WARN(node_->get_logger(),
                "[ArmCommander] No joint state data yet — call waitForJointStates() first");
    return false;
  }

  // Sanity check against the URDF position limits (warn only)
  for (size_t i = 0; i < positions.size(); ++i) {
    const size_t idx_in_arm = i % 4;
    if (positions[i] < kPositionLimits[idx_in_arm][0] ||
        positions[i] > kPositionLimits[idx_in_arm][1]) {
      RCLCPP_WARN(node_->get_logger(),
                  "[ArmCommander] Joint %s target %.3f rad exceeds URDF limit "
                  "[%.3f, %.3f]",
                  jointNames()[i].c_str(), positions[i],
                  kPositionLimits[idx_in_arm][0], kPositionLimits[idx_in_arm][1]);
    }
  }

  // ------------------------------------------------------------------
  // Build trajectory goal
  // ------------------------------------------------------------------
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory.joint_names = jointNames();
  goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(duration + 3.0);

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(positions.size());
  point.velocities.resize(positions.size());

  for (size_t i = 0; i < positions.size(); ++i) {
    point.positions[i] = positions[i];
    // Compute velocity so the joint arrives at the target in `duration`
    point.velocities[i] =
      (positions[i] - current_positions_[jointNames()[i]]) / duration;
  }
  point.time_from_start = rclcpp::Duration::from_seconds(duration);
  goal_msg.trajectory.points.push_back(std::move(point));

  // ------------------------------------------------------------------
  // Send goal without blocking; the caller spins and waits on the future
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "[ArmCommander] Sending trajectory goal ...");
  goal_future = action_client_->async_send_goal(goal_msg);
  return true;
}

// ============================================================================
//  asyncGetResult  (non-blocking result request)
// ============================================================================

ArmCommander::ResultFuture ArmCommander::asyncGetResult(
  const GoalHandle::SharedPtr & goal_handle)
{
  return action_client_->async_get_result(goal_handle);
}

// ============================================================================
//  sendPositionGoal  (blocking convenience wrapper)
// ============================================================================

bool ArmCommander::sendPositionGoal(const std::vector<double> & positions, double duration)
{
  SendGoalFuture send_goal_future;
  if (!asyncSendPositionGoal(positions, duration, send_goal_future)) {
    return false;
  }

  if (rclcpp::spin_until_future_complete(node_, send_goal_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[ArmCommander] Failed to send goal");
    return false;
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "[ArmCommander] Goal was rejected by controller");
    return false;
  }

  // ------------------------------------------------------------------
  // Wait for result
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "[ArmCommander] Goal accepted, waiting for execution ...");

  auto result_future = action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[ArmCommander] Interrupted while waiting for result");
    return false;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node_->get_logger(), "[ArmCommander] Trajectory execution succeeded!");
    return true;
  }

  RCLCPP_WARN(node_->get_logger(),
              "[ArmCommander] Trajectory execution failed (code %d)",
              static_cast<int>(result.code));
  return false;
}

}  // namespace robot_commander
