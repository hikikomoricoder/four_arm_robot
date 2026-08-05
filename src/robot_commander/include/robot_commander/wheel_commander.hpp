#ifndef ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_
#define ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <robot_interfaces/srv/group_state_manager.hpp>
#include <chrono>
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
 *   wc.driveTurn(0.1, 1.0);           // all wheels forward
 *   wc.driveLift(0.1, 3.0);           // lift mode (sinusoidal profile)
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
   * @brief All 4 wheels rotate forward at the same linear speed.
   *
   * Reserves the "wheel" group via /group_state_manager (mode "turn")
   * before moving and releases it after the movement completes.
   *
   * @param linear_speed  Desired ground speed (m/s), default 0.1.
   * @param duration      Movement duration (seconds), default 1.0.
   * @return true on success.
   */
  bool driveTurn(double linear_speed = 0.1, double duration = 1.0);

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

  /**
   * @brief Lift mode: all 4 wheels follow a half-sine velocity profile
   *        w(t) = w_peak * sin(pi * t / duration), t in [0, duration].
   *
   * The velocity starts and ends at 0 and peaks (w_peak) halfway through,
   * giving a smooth deformation motion while the veer joints hold the
   * lift angle (-45°).  Intended to run for the same duration as a
   * simultaneous arm operation (see CompoundCommander).
   *
   * Reserves the "wheel" group via /group_state_manager (mode "lift")
   * before moving and releases it after the movement completes.
   *
   * @param peak_linear_speed  Peak linear speed (m/s), default 0.1.
   * @param duration           Movement duration (seconds), default 3.0.
   * @return true on success.
   */
  bool driveLift(double peak_linear_speed = 0.1, double duration = 3.0);

  // -- low-level API (for advanced use) -----------------------------------

  /**
   * @brief Publish a custom velocity profile for a given duration, then stop.
   * @param velocities  4-element vector in controller joint order:
   *                    [wheel_joint_4, wheel_joint_3, wheel_joint_2, wheel_joint_1].
   * @param duration    Movement duration (seconds).
   * @return true on success.
   */
  bool driveWithVelocities(const std::vector<double> & velocities, double duration);

  /**
   * @brief Low-level lift profile: publish the half-sine velocity profile
   *        w(t) = (peak_linear_speed / WHEEL_RADIUS) * sin(pi * t / duration)
   *        to all four wheels at 50 Hz (sim time) for `duration`, then stop.
   *
   * Does NOT touch the /group_state_manager state lock — use driveLift()
   * for standalone operation, or call this from CompoundCommander which
   * manages the state lock itself.
   *
   * @param peak_linear_speed  Peak linear speed (m/s).
   * @param duration           Movement duration (seconds).
   * @return true on success.
   */
  bool driveWithLiftProfile(double peak_linear_speed, double duration);

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
