#include <robot_commander/veer_commander.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <algorithm>
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
//  setHomeState
// ============================================================================

bool VeerCommander::setHomeState(double duration)
{
  const auto & home = homePositions();

  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setHomeState: sending all veer joints to "
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

  // Step 1: return to home first, so the forward offset is always relative to home
  // -------------------------------------------------------------------
  // Compute a safe setHomeState duration based on the URDF velocity limit (1.0 rad/s)
  constexpr double kMaxJointVelocity = 1.0;  // from URDF <limit velocity="1.0"/>
  double max_home_delta = 0.0;
  for (const auto & jn : jointNames()) {
    auto it = current_positions_.find(jn);
    if (it != current_positions_.end()) {
      max_home_delta = std::max(max_home_delta, std::abs(it->second));
    }
  }
  const double go_home_duration = std::max(max_home_delta / kMaxJointVelocity, 1.0);

  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setForwardState: setHomeState first (%.1f s, max delta %.3f rad)",
              go_home_duration, max_home_delta);

  if (!setHomeState(go_home_duration)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] setForwardState: setHomeState failed");
    return false;
  }

  // Step 2: build forward targets relative to home
  // -------------------------------------------------------------------
  // Controller joint order: [arm_veer_joint_4, arm_veer_joint_3,
  //                          arm_veer_joint_2, arm_veer_joint_1]
  auto targets = homePositions();  // start from home {0,0,0,0}
  targets[0] -= M_PI_2;  // arm_veer_joint_4  -90° from home
  // arm_veer_joint_3 stays (0 from home)
  targets[2] -= M_PI_2;  // arm_veer_joint_2  -90° from home
  // arm_veer_joint_1 stays (0 from home)

  // Ensure enough time for the forward movement given velocity limits and
  // Gazebo PID tracking lag.  Empirical tuning shows 2.0 s is the minimum
  // for reliable convergence (position error < 0.15 rad).
  const double fwd_duration = std::max(duration, 2.0);

  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setForwardState: "
              "j1 stays (%.3f), j2 -pi/2 -> %.3f, "
              "j3 stays (%.3f), j4 -pi/2 -> %.3f  over %.1f s",
              targets[3], targets[2], targets[1], targets[0], fwd_duration);

  return sendPositionGoal(targets, fwd_duration);
}

// ============================================================================
//  setTurnState
// ============================================================================

bool VeerCommander::setTurnState(double duration)
{
  constexpr double kTurnAngle = M_PI_4;  // 45°

  // Relative to home (0 rad): every joint rotates +pi/4 -> pi/4
  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setTurnState: sending all veer joints to "
              "%.3f rad over %.1f s",
              kTurnAngle, duration);

  std::vector<double> targets(4, kTurnAngle);
  return sendPositionGoal(targets, duration);
}

// ============================================================================
//  setLiftState
// ============================================================================

bool VeerCommander::setLiftState(double duration)
{
  constexpr double kLiftAngle = -M_PI_4;  // -45°

  // Relative to home (0 rad): every joint rotates -pi/4 -> -pi/4
  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setLiftState: sending all veer joints to "
              "%.3f rad over %.1f s",
              kLiftAngle, duration);

  std::vector<double> targets(4, kLiftAngle);
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
