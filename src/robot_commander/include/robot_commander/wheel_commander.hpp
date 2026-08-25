#ifndef ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_
#define ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <robot_interfaces/srv/group_state_manager.hpp>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace robot_commander
{

/**
 * @brief Sends velocity commands to the wheel_controller
 *        (velocity_controllers/JointGroupVelocityController) via the
 *        /wheel_controller/commands topic.
 *
 * Usage:
 *   auto node = std::make_shared<rclcpp::Node>(...);
 *   WheelCommander wc(node);
 *   wc.driveForward(0.1, 1.0);        // differential forward
 *   wc.turnRightWithSpeed(0.1, 1.0);  // turn right in place
 *   wc.turnLeftWithSpeed(0.1, 1.0);   // turn left in place
 *   wc.turnByAngle(90.0, 0.1);        // turn in place by a given angle
 *
 * Linear speed (m/s) is converted internally to angular velocity (rad/s)
 * using the wheel radius (0.04 m).
 *
 * Each drive command forms a control loop with the /group_state_manager
 * service: before publishing it reserves the "wheel" group (status must
 * be "free", then set to "occupy" with the mode as position) and after
 * the movement finishes it releases the group back to "free" (position
 * unchanged).  If the wheel group is not free, the command is rejected.
 */
class WheelCommander
{
public:
  /** Wheel radius from far_common_properties.xml.xacro */
  static constexpr double WHEEL_RADIUS = 0.04;  // meters

  /**
   * Adjacent arm-base separation in the home pose (m), derived from the
   * URDF exactly like CompoundCommander's baseSeparation(0, 0) — see
   * docs/src/pkg/robot_commander/commander.md:
   *   L = group_joint x-offset + J3->J5 link * sin(pi/4)
   *                          + J5->J7 link * sin(3*pi/4)
   *     = 0.05 + 0.31 * sqrt(2)  ≈ 0.4884 m
   */
  static constexpr double HOME_BASE_SEPARATION = 0.05 + 0.31 * M_SQRT2;

  /**
   * In-place turn radius (m): in the home pose the four coplanar arm bases
   * (each carrying one veer + wheel) form a square of side
   * HOME_BASE_SEPARATION, each rotated 90° in yaw from its neighbour, so
   * the yaw axis is the square centre and every wheel contact point is at
   * radius L / sqrt(2).  Body yaw rate = wheel ground speed / TURN_RADIUS.
   */
  static constexpr double TURN_RADIUS = HOME_BASE_SEPARATION / M_SQRT2;

  /**
   * @param node  A fully initialised rclcpp node (use_sim_time and other
   *              parameters should be set on it before construction).
   * @param command_topic  Velocity command topic, defaults to
   *                       "/wheel_controller/commands".
   */
  explicit WheelCommander(
    rclcpp::Node::SharedPtr node,
    const std::string & command_topic = "/wheel_controller/commands");

  virtual ~WheelCommander() = default;

  // -- wait helpers -------------------------------------------------------

  /**
   * @brief Block until all four wheel joint positions are received on
   *        /joint_states (or until timeout).
   * @return true on success, false on timeout.
   */
  bool waitForJointStates(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  // -- drive commands -----------------------------------------------------

  /**
   * @brief Turn right in place: all 4 wheels rotate at the same linear speed.
   *
   * Pre-checks: veer position must be "turn", veer status must be "free",
   * wheel status must be "free".  Reserves the "wheel" group via
   * /group_state_manager (position "turn") before moving and releases it
   * after the movement completes.
   *
   * @param linear_speed  Desired ground speed (m/s), default 0.1.
   * @param duration      Movement duration (seconds), default 1.0.
   * @return true on success.
   */
  bool turnRightWithSpeed(double linear_speed = 0.1, double duration = 1.0);

  /**
   * @brief Turn left in place: all 4 wheels rotate at the same linear speed
   *        in the opposite direction of turnRightWithSpeed.
   *
   * Pre-checks: veer position must be "turn", veer status must be "free",
   * wheel status must be "free".  Reserves the "wheel" group via
   * /group_state_manager (position "turn") before moving and releases it
   * after the movement completes.
   *
   * @param linear_speed  Desired ground speed (m/s), default 0.1.
   * @param duration      Movement duration (seconds), default 1.0.
   * @return true on success.
   */
  bool turnLeftWithSpeed(double linear_speed = 0.1, double duration = 1.0);

  /**
   * @brief Turn in place by a specified angle.
   *
   * The duration is derived from the turn geometry so that the body yaws
   * by the requested angle:
   *   duration = |angle| * TURN_RADIUS / linear_speed
   * Positive wheel speeds rotate the robot clockwise, therefore a positive
   * angle turns counter-clockwise (left, +yaw) and a negative angle turns
   * clockwise (right, -yaw).
   *
   * Pre-checks (single get_all): veer position must be "turn", veer status
   * "free", arm position "home", arm status "free" and wheel status "free".
   * Reserves the "wheel" group via /group_state_manager (position "turn",
   * status "occupy") before moving and releases it (status "free") after
   * the movement completes.
   *
   * @param turn_angle_deg  Desired body yaw change in degrees
   *                        (positive = counter-clockwise / left).
   * @param linear_speed    Wheel ground speed (m/s), default 0.1.
   * @return true on success.
   */
  bool turnByAngle(double turn_angle_deg, double linear_speed = 0.1);

  /**
   * @brief Differential steering: wheel_joint_1,2 forward,
   *        wheel_joint_3,4 reverse.
   *
   * Reserves the "wheel" group via /group_state_manager (mode "forward")
   * before moving and releases it after the movement completes.
   *
   * @param linear_speed  Base linear speed (m/s), default 0.1.
   * @param duration      Movement duration (seconds), default 1.0.
   * @return true on success.
   */
  bool driveForward(double linear_speed = 0.1, double duration = 1.0);

  // -- low-level API (for advanced use) -----------------------------------

  /**
   * @brief Publish a custom velocity profile for a given duration, then stop.
   *
   * Before publishing, waits for at least one subscriber on the command
   * topic (3 s sim-time timeout).
   *
   * @param velocities  4-element vector in controller joint order:
   *                    [wheel_joint_4, wheel_joint_3, wheel_joint_2, wheel_joint_1].
   * @param duration    Movement duration (seconds).
   * @return true on success.
   */
  bool driveWithVelocities(const std::vector<double> & velocities, double duration);

  /**
   * @brief Publish the same angular velocity to all four wheels immediately
   *        (low-level, no state lock, no waiting).  Used by
   *        CompoundCommander to drive the lift profile while a blocking
   *        arm trajectory executes concurrently on the same node.
   * @param angular_velocity  Wheel angular velocity (rad/s).
   */
  void publishLiftVelocity(double angular_velocity);

  /** Controller joint order (read-only) */
  static const std::vector<std::string> & jointNames()
  {
    static const std::vector<std::string> kNames{
      "wheel_joint_4", "wheel_joint_3", "wheel_joint_2", "wheel_joint_1"};
    return kNames;
  }

private:
  // -- state management helpers ------------------------------------------

  /**
   * @brief Reserve the wheel group via the group_state_manager service.
   *
   * Checks that the wheel status is "free", then sets it to "occupy"
   * and records the requested mode as the current position.
   *
   * @param mode  The mode being requested ("forward", "turn", "lift").
   * @return true if the wheel group was successfully reserved.
   */
  bool reserveWheel(const std::string & mode);

  /**
   * @brief Release the wheel group back to "free" status.
   *
   * The recorded position is left unchanged.
   */
  void releaseWheel();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Client<robot_interfaces::srv::GroupStateManager>::SharedPtr state_manager_client_;
  std::map<std::string, double> current_positions_;
  std::string command_topic_;
  bool reserved_ = false;
};

}  // namespace robot_commander

#endif  // ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_
