#include <robot_commander/mobile_base_commander.hpp>

#include <chrono>
#include <cmath>
#include <string>

namespace robot_commander
{

// ============================================================================
//  Constructor
// ============================================================================

MobileBaseCommander::MobileBaseCommander(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  veer_commander_(node_),
  wheel_commander_(node_)
{
  // Create the group_state_manager service client (overall pre-check)
  state_manager_client_ =
    node_->create_client<robot_interfaces::srv::GroupStateManager>("group_state_manager");
}

// ============================================================================
//  waitForJointStates
// ============================================================================

bool MobileBaseCommander::waitForJointStates(const std::chrono::seconds & timeout)
{
  return veer_commander_.waitForJointStates(timeout) &&
         wheel_commander_.waitForJointStates(timeout);
}

// ============================================================================
//  waitForActionServer
// ============================================================================

bool MobileBaseCommander::waitForActionServer(const std::chrono::seconds & timeout)
{
  return veer_commander_.waitForActionServer(timeout);
}

// ============================================================================
//  checkPreconditions   (single get_all)
// ============================================================================

bool MobileBaseCommander::checkPreconditions()
{
  if (!state_manager_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MobileBaseCommander] group_state_manager service not available");
    return false;
  }

  auto get_req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  get_req->command = "get_all";

  auto get_future = state_manager_client_->async_send_request(get_req);
  if (rclcpp::spin_until_future_complete(node_, get_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MobileBaseCommander] Failed to call group_state_manager (get_all)");
    return false;
  }

  auto get_resp = get_future.get();
  if (!get_resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MobileBaseCommander] get_all failed: %s", get_resp->message.c_str());
    return false;
  }

  if (get_resp->arm_position != "home") {
    RCLCPP_WARN(node_->get_logger(),
                "[MobileBaseCommander] arm position is '%s' (not 'home'), cannot move",
                get_resp->arm_position.c_str());
    return false;
  }
  if (get_resp->arm_status != "free") {
    RCLCPP_WARN(node_->get_logger(),
                "[MobileBaseCommander] arm status is '%s' (not 'free'), cannot move",
                get_resp->arm_status.c_str());
    return false;
  }
  if (get_resp->veer_status != "free") {
    RCLCPP_WARN(node_->get_logger(),
                "[MobileBaseCommander] veer status is '%s' (not 'free'), cannot move",
                get_resp->veer_status.c_str());
    return false;
  }
  if (get_resp->wheel_status != "free") {
    RCLCPP_WARN(node_->get_logger(),
                "[MobileBaseCommander] wheel status is '%s' (not 'free'), cannot move",
                get_resp->wheel_status.c_str());
    return false;
  }

  return true;
}

// ============================================================================
//  moveToPosition
// ============================================================================

bool MobileBaseCommander::moveToPosition(double x, double y, double linear_speed)
{
  if (linear_speed <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MobileBaseCommander] linear speed must be > 0 (got %.3f)",
                 linear_speed);
    return false;
  }

  const double distance = std::hypot(x, y);
  if (distance < 1e-6) {
    RCLCPP_INFO(node_->get_logger(),
                "[MobileBaseCommander] target is the current position, nothing to do");
    return true;
  }

  // Target heading relative to the current body frame (x forward, y left).
  // Positive = counter-clockwise / left, matching turnByAngle's convention.
  const double yaw_deg = std::atan2(y, x) * 180.0 / M_PI;

  RCLCPP_INFO(node_->get_logger(),
              "[MobileBaseCommander] moveToPosition: target (%.3f, %.3f) m -> "
              "heading %.1f deg, distance %.3f m @ %.3f m/s",
              x, y, yaw_deg, distance, linear_speed);

  // ---- Pre-check (get_all): arm home/free, veer free, wheel free -----------
  if (!checkPreconditions()) {
    return false;
  }

  // ---- Phase 1: turn in place to the target heading ------------------------
  // Skipped when the target lies straight ahead (no veer reconfiguration).
  if (std::abs(yaw_deg) > 1e-3) {
    RCLCPP_INFO(node_->get_logger(),
                "[MobileBaseCommander] Phase 1/2: turn in place by %.1f deg",
                yaw_deg);
    if (!veer_commander_.setTurnState() ||
        !wheel_commander_.turnByAngle(yaw_deg, linear_speed))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[MobileBaseCommander] Phase 1 (in-place turn) failed");
      veer_commander_.setHomeState();  // best-effort restore
      return false;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "[MobileBaseCommander] Phase 1/2: heading already aligned, skipping turn");
  }

  // ---- Phase 2: drive straight to the target -------------------------------
  const double forward_duration = distance / linear_speed;
  RCLCPP_INFO(node_->get_logger(),
              "[MobileBaseCommander] Phase 2/2: drive forward %.3f m (%.2f s)",
              distance, forward_duration);
  if (!veer_commander_.setForwardState() ||
      !wheel_commander_.driveForward(linear_speed, forward_duration))
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MobileBaseCommander] Phase 2 (straight drive) failed");
    veer_commander_.setHomeState();  // best-effort restore
    return false;
  }

  // ---- Finish: restore the neutral veer configuration -----------------------
  RCLCPP_INFO(node_->get_logger(),
              "[MobileBaseCommander] Movement done, restoring veer home");
  return veer_commander_.setHomeState();
}

}  // namespace robot_commander
