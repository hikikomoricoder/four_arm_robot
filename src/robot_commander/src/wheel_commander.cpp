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
//  reserveWheel   (state management helper)
// ============================================================================

bool WheelCommander::reserveWheel(const std::string & mode)
{
  if (reserved_) {
    return true;  // already reserved (e.g. nested call)
  }

  if (!state_manager_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] group_state_manager service not available");
    return false;
  }

  // Step 1: check current status
  auto get_req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  get_req->command = "get_group";
  get_req->group_name = "wheel";

  auto get_future = state_manager_client_->async_send_request(get_req);
  if (rclcpp::spin_until_future_complete(node_, get_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] Failed to call group_state_manager (get_group)");
    return false;
  }

  auto get_resp = get_future.get();
  if (!get_resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] get_group failed: %s", get_resp->message.c_str());
    return false;
  }

  if (get_resp->wheel_status != "free") {
    RCLCPP_WARN(node_->get_logger(),
                "[WheelCommander] wheel status is '%s' (not 'free'), cannot execute",
                get_resp->wheel_status.c_str());
    return false;
  }

  // Step 2: reserve — set status=occupy, position=mode
  auto set_req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  set_req->command = "set_group";
  set_req->group_name = "wheel";
  set_req->position_name = mode;
  set_req->status_name = "occupy";

  auto set_future = state_manager_client_->async_send_request(set_req);
  if (rclcpp::spin_until_future_complete(node_, set_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] Failed to call group_state_manager (set_group)");
    return false;
  }

  auto set_resp = set_future.get();
  if (!set_resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] set_group failed: %s", set_resp->message.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[WheelCommander] wheel reserved: mode=%s, status=occupy",
              mode.c_str());
  reserved_ = true;
  return true;
}

// ============================================================================
//  releaseWheel   (state management helper)
// ============================================================================

void WheelCommander::releaseWheel()
{
  if (!reserved_) {
    return;  // not currently reserved
  }

  if (!state_manager_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] group_state_manager service not available for release");
    reserved_ = false;
    return;
  }

  auto req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  req->command = "set_group";
  req->group_name = "wheel";
  req->position_name = "";  // leave position unchanged
  req->status_name = "free";

  auto future = state_manager_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] Failed to release wheel state");
  } else {
    auto resp = future.get();
    if (resp->success) {
      RCLCPP_INFO(node_->get_logger(),
                  "[WheelCommander] wheel released: status=free");
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "[WheelCommander] Release failed: %s", resp->message.c_str());
    }
  }

  reserved_ = false;
}

// ============================================================================
//  turnRightWithSpeed
// ============================================================================

bool WheelCommander::turnRightWithSpeed(double linear_speed, double duration)
{
  // ---- Pre-check: veer must be in "turn" position and free -----------
  if (!state_manager_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] group_state_manager service not available");
    return false;
  }

  auto veer_req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  veer_req->command = "get_group";
  veer_req->group_name = "veer";

  auto veer_future = state_manager_client_->async_send_request(veer_req);
  if (rclcpp::spin_until_future_complete(node_, veer_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] Failed to query veer state");
    return false;
  }

  auto veer_resp = veer_future.get();
  if (!veer_resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] get_group veer failed: %s",
                 veer_resp->message.c_str());
    return false;
  }

  if (veer_resp->veer_position != "turn") {
    RCLCPP_WARN(node_->get_logger(),
                "[WheelCommander] veer position is '%s' (not 'turn'), cannot turn",
                veer_resp->veer_position.c_str());
    return false;
  }

  if (veer_resp->veer_status != "free") {
    RCLCPP_WARN(node_->get_logger(),
                "[WheelCommander] veer status is '%s' (not 'free'), cannot turn",
                veer_resp->veer_status.c_str());
    return false;
  }

  // ---- Reserve wheel --------------------------------------------------
  if (!reserveWheel("turn")) {
    return false;
  }

  const double angular_vel = linear_speed / WHEEL_RADIUS;

  RCLCPP_INFO(node_->get_logger(),
              "[WheelCommander] turnRightWithSpeed: all wheels @ +%.3f rad/s (%.3f m/s) "
              "for %.1f s",
              angular_vel, linear_speed, duration);

  const std::vector<double> velocities = {
    angular_vel,   // wheel_joint_4
    angular_vel,   // wheel_joint_3
    angular_vel,   // wheel_joint_2
    angular_vel    // wheel_joint_1
  };
  bool ok = driveWithVelocities(velocities, duration);
  releaseWheel();
  return ok;
}

// ============================================================================
//  turnLeftWithSpeed
// ============================================================================

bool WheelCommander::turnLeftWithSpeed(double linear_speed, double duration)
{
  // ---- Pre-check: veer must be in "turn" position and free -----------
  if (!state_manager_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] group_state_manager service not available");
    return false;
  }

  auto veer_req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  veer_req->command = "get_group";
  veer_req->group_name = "veer";

  auto veer_future = state_manager_client_->async_send_request(veer_req);
  if (rclcpp::spin_until_future_complete(node_, veer_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] Failed to query veer state");
    return false;
  }

  auto veer_resp = veer_future.get();
  if (!veer_resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[WheelCommander] get_group veer failed: %s",
                 veer_resp->message.c_str());
    return false;
  }

  if (veer_resp->veer_position != "turn") {
    RCLCPP_WARN(node_->get_logger(),
                "[WheelCommander] veer position is '%s' (not 'turn'), cannot turn",
                veer_resp->veer_position.c_str());
    return false;
  }

  if (veer_resp->veer_status != "free") {
    RCLCPP_WARN(node_->get_logger(),
                "[WheelCommander] veer status is '%s' (not 'free'), cannot turn",
                veer_resp->veer_status.c_str());
    return false;
  }

  // ---- Reserve wheel --------------------------------------------------
  if (!reserveWheel("turn")) {
    return false;
  }

  const double angular_vel = linear_speed / WHEEL_RADIUS;

  RCLCPP_INFO(node_->get_logger(),
              "[WheelCommander] turnLeftWithSpeed: all wheels @ -%.3f rad/s (%.3f m/s) "
              "for %.1f s",
              angular_vel, linear_speed, duration);

  const std::vector<double> velocities = {
    -angular_vel,   // wheel_joint_4
    -angular_vel,   // wheel_joint_3
    -angular_vel,   // wheel_joint_2
    -angular_vel    // wheel_joint_1
  };
  bool ok = driveWithVelocities(velocities, duration);
  releaseWheel();
  return ok;
}

// ============================================================================
//  driveForward
// ============================================================================

bool WheelCommander::driveForward(double linear_speed, double duration)
{
  if (!reserveWheel("forward")) {
    return false;
  }

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
  bool ok = driveWithVelocities(velocities, duration);
  releaseWheel();
  return ok;
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

// ============================================================================
//  publishLiftVelocity  (low-level immediate publish)
// ============================================================================

void WheelCommander::publishLiftVelocity(double angular_velocity)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {
    angular_velocity,   // wheel_joint_4
    angular_velocity,   // wheel_joint_3
    angular_velocity,   // wheel_joint_2
    angular_velocity    // wheel_joint_1
  };
  velocity_pub_->publish(msg);
}

}  // namespace robot_commander
