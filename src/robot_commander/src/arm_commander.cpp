#include <robot_commander/arm_commander.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_commander
{

namespace
{
// Max joint velocity from joint_limits.yaml (1.0 rad/s for all arm joints)
constexpr double kMaxJointVelocity = 1.0;  // rad/s

// URDF position limits (rad):
//   arm_joint_1_x: [-3.14, 3.14]
//   arm_joint_3/5/7_x: [-2.35, 2.35]
// Indexed by the position inside each per-arm block [j1, j3, j5, j7].
constexpr double kPositionLimits[4][2] = {
  {-3.14, 3.14},
  {-2.35, 2.35},
  {-2.35, 2.35},
  {-2.35, 2.35}};

// Minimum duration of the offset step, tuned for reliable convergence
// under Gazebo physics (same as the veer forward preset).
constexpr double kMinOffsetDuration = 2.0;  // s
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
//  setHomeState
// ============================================================================

bool ArmCommander::setHomeState(double duration)
{
  const auto & home = homePositions();

  RCLCPP_INFO(node_->get_logger(),
              "[ArmCommander] setHomeState: sending all 16 arm joints to "
              "0 rad over %.1f s",
              duration);

  return sendPositionGoal(home, duration);
}

// ============================================================================
//  homeThenOffset  (two-step motion helper)
// ============================================================================

bool ArmCommander::homeThenOffset(const std::vector<double> & offsets, double duration)
{
  // Ensure we have joint state data
  if (!current_positions_.count("arm_joint_1_1")) {
    RCLCPP_WARN(node_->get_logger(),
                "[ArmCommander] No joint state data yet — call waitForJointStates() first");
    return false;
  }

  // Step 1: return to home first, so the offset is always relative to the
  // initial (home) state.
  // -------------------------------------------------------------------
  // Compute a safe setHomeState duration based on the joint velocity limit.
  double max_home_delta = 0.0;
  for (const auto & jn : jointNames()) {
    auto it = current_positions_.find(jn);
    if (it != current_positions_.end()) {
      max_home_delta = std::max(max_home_delta, std::abs(it->second));
    }
  }
  const double go_home_duration = std::max(max_home_delta / kMaxJointVelocity, 1.0);

  RCLCPP_INFO(node_->get_logger(),
              "[ArmCommander] homeThenOffset: setHomeState first (%.1f s, max delta %.3f rad)",
              go_home_duration, max_home_delta);

  if (!setHomeState(go_home_duration)) {
    RCLCPP_ERROR(node_->get_logger(), "[ArmCommander] homeThenOffset: setHomeState failed");
    return false;
  }

  // Step 2: targets = home (0) + offsets, in controller joint order.
  // -------------------------------------------------------------------
  // Ensure enough time for the offset movement given velocity limits and
  // Gazebo PID tracking lag (same floor as the veer forward preset).
  const double offset_duration = std::max(duration, kMinOffsetDuration);

  RCLCPP_INFO(node_->get_logger(),
              "[ArmCommander] homeThenOffset: moving to offsets "
              "[%.3f, %.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f | "
              "%.3f, %.3f, %.3f, %.3f | %.3f, %.3f, %.3f, %.3f] rad over %.1f s",
              offsets[0], offsets[1], offsets[2], offsets[3],
              offsets[4], offsets[5], offsets[6], offsets[7],
              offsets[8], offsets[9], offsets[10], offsets[11],
              offsets[12], offsets[13], offsets[14], offsets[15],
              offset_duration);

  return sendPositionGoal(offsets, offset_duration);
}

// ============================================================================
//  setLowState
// ============================================================================

bool ArmCommander::setLowState(double duration)
{
  // Per-arm block [j1, j3, j5, j7]; all four arms move identically:
  //   j3 -pi/4, j5 +pi/2, j7 -pi/4 (mimic of j5: -1/2 * (+pi/2))
  std::vector<double> offsets(16, 0.0);
  for (int arm = 0; arm < 4; ++arm) {
    const size_t base = static_cast<size_t>(arm) * 4;
    offsets[base + 1] = -M_PI_4;   // arm_joint_3_x
    offsets[base + 2] = M_PI_2;    // arm_joint_5_x
    offsets[base + 3] = -M_PI_4;   // arm_joint_7_x
  }

  RCLCPP_INFO(node_->get_logger(),
              "[ArmCommander] setLowState: j3 -pi/4, j5 +pi/2, j7 -pi/4 for all arms");
  return homeThenOffset(offsets, duration);
}

// ============================================================================
//  setHighState
// ============================================================================

bool ArmCommander::setHighState(double duration)
{
  // Per-arm block [j1, j3, j5, j7]; all four arms move identically:
  //   j3 +pi/8, j5 -pi/4, j7 +pi/8 (mimic of j5: -1/2 * (-pi/4))
  std::vector<double> offsets(16, 0.0);
  for (int arm = 0; arm < 4; ++arm) {
    const size_t base = static_cast<size_t>(arm) * 4;
    offsets[base + 1] = M_PI / 8.0;   // arm_joint_3_x
    offsets[base + 2] = -M_PI_4;      // arm_joint_5_x
    offsets[base + 3] = M_PI / 8.0;   // arm_joint_7_x
  }

  RCLCPP_INFO(node_->get_logger(),
              "[ArmCommander] setHighState: j3 +pi/8, j5 -pi/4, j7 +pi/8 for all arms");
  return homeThenOffset(offsets, duration);
}

// ============================================================================
//  setRhombus1State
// ============================================================================

bool ArmCommander::setRhombus1State(double duration)
{
  // Only arm_joint_1_x moves, forming a diamond pattern:
  //   j1_1 +pi/4, j1_2 -pi/4, j1_3 +pi/4, j1_4 -pi/4
  std::vector<double> offsets(16, 0.0);
  offsets[0]  = M_PI_4;   // arm_joint_1_1
  offsets[4]  = -M_PI_4;  // arm_joint_1_2
  offsets[8]  = M_PI_4;   // arm_joint_1_3
  offsets[12] = -M_PI_4;  // arm_joint_1_4

  RCLCPP_INFO(node_->get_logger(),
              "[ArmCommander] setRhombus1State: j1 = +pi/4, -pi/4, +pi/4, -pi/4");
  return homeThenOffset(offsets, duration);
}

// ============================================================================
//  setRhombus2State
// ============================================================================

bool ArmCommander::setRhombus2State(double duration)
{
  // Only arm_joint_1_x moves, opposite diamond pattern:
  //   j1_1 -pi/4, j1_2 +pi/4, j1_3 -pi/4, j1_4 +pi/4
  std::vector<double> offsets(16, 0.0);
  offsets[0]  = -M_PI_4;  // arm_joint_1_1
  offsets[4]  = M_PI_4;   // arm_joint_1_2
  offsets[8]  = -M_PI_4;  // arm_joint_1_3
  offsets[12] = M_PI_4;   // arm_joint_1_4

  RCLCPP_INFO(node_->get_logger(),
              "[ArmCommander] setRhombus2State: j1 = -pi/4, +pi/4, -pi/4, +pi/4");
  return homeThenOffset(offsets, duration);
}

// ============================================================================
//  sendPositionGoal  (core implementation)
// ============================================================================

bool ArmCommander::sendPositionGoal(const std::vector<double> & positions, double duration)
{
  if (!rclcpp::ok()) {
    return false;
  }
  if (positions.size() != jointNames().size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[ArmCommander] sendPositionGoal: expected %zu positions, got %zu",
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
  // Send goal
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "[ArmCommander] Sending trajectory goal ...");

  auto send_goal_future = action_client_->async_send_goal(goal_msg);
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
