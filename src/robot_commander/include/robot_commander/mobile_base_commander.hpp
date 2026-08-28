#ifndef ROBOT_COMMANDER__MOBILE_BASE_COMMANDER_HPP_
#define ROBOT_COMMANDER__MOBILE_BASE_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/srv/group_state_manager.hpp>
#include <robot_commander/veer_commander.hpp>
#include <robot_commander/wheel_commander.hpp>
#include <chrono>
#include <memory>

namespace robot_commander
{

/**
 * @brief Moves the mobile base to a position relative to the current one
 *        by composing the existing VeerCommander and WheelCommander
 *        capabilities (veer turn, wheel turn, wheel forward).
 *
 * Movement flow for moveToPosition(x, y):
 *   1. Pre-check (single get_all on /group_state_manager) — only executes
 *      when arm position == "home", arm status == "free",
 *      veer status == "free" and wheel status == "free".  The veer
 *      position is intentionally not constrained because this commander
 *      drives the veer configuration itself.
 *   2. Turn in place to the target heading:
 *        veer  -> setTurnState()          (all four veer joints +pi/4)
 *        wheel -> turnByAngle(yaw, speed) (yaw = atan2(y, x) in degrees)
 *      Skipped entirely when the target lies straight ahead (yaw == 0).
 *   3. Drive straight to the target:
 *        veer  -> setForwardState()                  (j2/j4 -> -pi/2)
 *        wheel -> driveForward(speed, distance/speed)
 *      with distance = hypot(x, y).
 *   4. Restore the neutral configuration: veer -> setHomeState().
 *
 * Every sub-command keeps its own occupy/free state lock with
 * /group_state_manager (VeerCommander reserves/releases the "veer" group,
 * WheelCommander the "wheel" group); this commander only adds the overall
 * pre-check, the phase orchestration and the final veer home restore.
 *
 * Usage:
 *   auto node = std::make_shared<rclcpp::Node>(...);
 *   MobileBaseCommander mbc(node);
 *   mbc.waitForJointStates();
 *   mbc.waitForActionServer();
 *   mbc.moveToPosition(0.5, 0.3);   // 0.5 m forward, 0.3 m to the left
 */
class MobileBaseCommander
{
public:
  /**
   * @param node  A fully initialised rclcpp node (use_sim_time and other
   *              parameters should be set on it before construction).
   */
  explicit MobileBaseCommander(rclcpp::Node::SharedPtr node);

  virtual ~MobileBaseCommander() = default;

  // -- wait helpers -------------------------------------------------------

  /**
   * @brief Block until all four veer joint and four wheel joint positions
   *        are received on /joint_states (or until timeout, per group).
   * @return true on success, false on timeout.
   */
  bool waitForJointStates(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  /**
   * @brief Block until the veer controller action server is available.
   * @return true on success, false on timeout.
   */
  bool waitForActionServer(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  // -- movement command -----------------------------------------------------

  /**
   * @brief Move to a position relative to the current one: first turn in
   *        place to the target heading, then drive straight to the target.
   *
   * The target is expressed in the robot's current body frame:
   * x forward, y to the left (m).  The heading to turn to is
   * atan2(y, x) (positive = counter-clockwise / left, matching
   * WheelCommander::turnByAngle) and the straight-line travel distance
   * is hypot(x, y).
   *
   * Pre-check (single get_all): arm position "home", arm status "free",
   * veer status "free", wheel status "free" — otherwise the command is
   * rejected and nothing moves.
   *
   * After the movement (or on failure, best effort) the veer joints are
   * returned to the home configuration.
   *
   * @param x             Forward displacement (m).
   * @param y             Leftward displacement (m).
   * @param linear_speed  Wheel ground speed (m/s) for both the in-place
   *                      turn and the straight drive, default 0.1.
   * @return true if the whole movement completed successfully.
   */
  bool moveToPosition(double x, double y, double linear_speed = 0.1);

private:
  // -- state management helpers ------------------------------------------

  /**
   * @brief Pre-check via /group_state_manager (get_all): arm position
   *        "home", arm status "free", veer status "free", wheel status
   *        "free".
   * @return true if the movement may execute.
   */
  bool checkPreconditions();

  rclcpp::Node::SharedPtr node_;
  VeerCommander veer_commander_;
  WheelCommander wheel_commander_;
  rclcpp::Client<robot_interfaces::srv::GroupStateManager>::SharedPtr state_manager_client_;
};

}  // namespace robot_commander

#endif  // ROBOT_COMMANDER__MOBILE_BASE_COMMANDER_HPP_
