#ifndef ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_
#define ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
 *
 * Linear speed (m/s) is converted internally to angular velocity (rad/s)
 * using the wheel radius (0.04 m).
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
   * @param linear_speed  Desired ground speed (m/s), default 0.1.
   * @param duration      Movement duration (seconds), default 1.0.
   * @return true on success.
   */
  bool driveTurn(double linear_speed = 0.1, double duration = 1.0);

  /**
   * @brief Differential steering: wheel_joint_1,2 forward,
   *        wheel_joint_3,4 reverse.
   * @param linear_speed  Base linear speed (m/s), default 0.1.
   * @param duration      Movement duration (seconds), default 1.0.
   * @return true on success.
   */
  bool driveForward(double linear_speed = 0.1, double duration = 1.0);

  // -- low-level API (for advanced use) -----------------------------------

  /**
   * @brief Publish a custom velocity profile for a given duration, then stop.
   * @param velocities  4-element vector in controller joint order:
   *                    [wheel_joint_4, wheel_joint_3, wheel_joint_2, wheel_joint_1].
   * @param duration    Movement duration (seconds).
   * @return true on success.
   */
  bool driveWithVelocities(const std::vector<double> & velocities, double duration);

  /** Controller joint order (read-only) */
  static const std::vector<std::string> & jointNames()
  {
    static const std::vector<std::string> kNames{
      "wheel_joint_4", "wheel_joint_3", "wheel_joint_2", "wheel_joint_1"};
    return kNames;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::map<std::string, double> current_positions_;
  std::string command_topic_;
};

}  // namespace robot_commander

#endif  // ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_
