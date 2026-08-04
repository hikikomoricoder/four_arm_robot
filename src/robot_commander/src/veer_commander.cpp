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

  // Create the group_state_manager service client
  state_manager_client_ =
    node_->create_client<robot_interfaces::srv::GroupStateManager>("group_state_manager");

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
//  reserveVeer   (state management helper)
// ============================================================================

bool VeerCommander::reserveVeer(const std::string & mode)
{
  if (reserved_) {
    return true;  // already reserved (e.g. nested call from setForwardState)
  }

  if (!state_manager_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] group_state_manager service not available");
    return false;
  }

  // Step 1: check current status
  auto get_req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  get_req->command = "get_group";
  get_req->group_name = "veer";

  auto get_future = state_manager_client_->async_send_request(get_req);
  if (rclcpp::spin_until_future_complete(node_, get_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] Failed to call group_state_manager (get_group)");
    return false;
  }

  auto get_resp = get_future.get();
  if (!get_resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] get_group failed: %s", get_resp->message.c_str());
    return false;
  }

  if (get_resp->veer_status != "free") {
    RCLCPP_WARN(node_->get_logger(),
                "[VeerCommander] veer status is '%s' (not 'free'), cannot execute",
                get_resp->veer_status.c_str());
    return false;
  }

  // Step 2: reserve — set status=occupy, position=mode
  auto set_req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  set_req->command = "set_group";
  set_req->group_name = "veer";
  set_req->position_name = mode;
  set_req->status_name = "occupy";

  auto set_future = state_manager_client_->async_send_request(set_req);
  if (rclcpp::spin_until_future_complete(node_, set_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] Failed to call group_state_manager (set_group)");
    return false;
  }

  auto set_resp = set_future.get();
  if (!set_resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] set_group failed: %s", set_resp->message.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] veer reserved: mode=%s, status=occupy",
              mode.c_str());
  reserved_ = true;
  return true;
}

// ============================================================================
//  releaseVeer   (state management helper)
// ============================================================================

void VeerCommander::releaseVeer()
{
  if (!reserved_) {
    return;  // not currently reserved
  }

  if (!state_manager_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] group_state_manager service not available for release");
    reserved_ = false;
    return;
  }

  auto req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  req->command = "set_group";
  req->group_name = "veer";
  req->position_name = "";  // leave position unchanged
  req->status_name = "free";

  auto future = state_manager_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[VeerCommander] Failed to release veer state");
  } else {
    auto resp = future.get();
    if (resp->success) {
      RCLCPP_INFO(node_->get_logger(),
                  "[VeerCommander] veer released: status=free");
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "[VeerCommander] Release failed: %s", resp->message.c_str());
    }
  }

  reserved_ = false;
}

// ============================================================================
//  setHomeState
// ============================================================================

bool VeerCommander::setHomeState(double duration)
{
  if (!reserveVeer("home")) {
    return false;
  }

  const auto & home = homePositions();

  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setHomeState: sending all veer joints to "
              "[%.3f, %.3f, %.3f, %.3f] rad over %.1f s",
              home[0], home[1], home[2], home[3], duration);

  bool ok = sendPositionGoal(home, duration);
  releaseVeer();
  return ok;
}

// ============================================================================
//  setForwardState
// ============================================================================

bool VeerCommander::setForwardState(double duration)
{
  if (!reserveVeer("forward")) {
    return false;
  }

  // Ensure we have joint state data
  if (!current_positions_.count("arm_veer_joint_1")) {
    RCLCPP_WARN(node_->get_logger(),
                "[VeerCommander] No joint state data yet — call waitForJointStates() first");
    releaseVeer();
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
    releaseVeer();
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

  bool ok = sendPositionGoal(targets, fwd_duration);
  releaseVeer();
  return ok;
}

// ============================================================================
//  setTurnState
// ============================================================================

bool VeerCommander::setTurnState(double duration)
{
  if (!reserveVeer("turn")) {
    return false;
  }

  constexpr double kTurnAngle = M_PI_4;  // 45°

  // Relative to home (0 rad): every joint rotates +pi/4 -> pi/4
  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setTurnState: sending all veer joints to "
              "%.3f rad over %.1f s",
              kTurnAngle, duration);

  std::vector<double> targets(4, kTurnAngle);
  bool ok = sendPositionGoal(targets, duration);
  releaseVeer();
  return ok;
}

// ============================================================================
//  setLiftState
// ============================================================================

bool VeerCommander::setLiftState(double duration)
{
  if (!reserveVeer("lift")) {
    return false;
  }

  constexpr double kLiftAngle = -M_PI_4;  // -45°

  // Relative to home (0 rad): every joint rotates -pi/4 -> -pi/4
  RCLCPP_INFO(node_->get_logger(),
              "[VeerCommander] setLiftState: sending all veer joints to "
              "%.3f rad over %.1f s",
              kLiftAngle, duration);

  std::vector<double> targets(4, kLiftAngle);
  bool ok = sendPositionGoal(targets, duration);
  releaseVeer();
  return ok;
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
