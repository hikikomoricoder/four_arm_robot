#ifndef ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_
#define ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace robot_commander
{

/**
 * @brief Sends velocity-based trajectory commands to the wheel_controller
 *        (joint_trajectory_controller) via its FollowJointTrajectory action.
 *
 * Usage:
 *   auto node = std::make_shared<rclcpp::Node>(...);
 *   WheelCommander wc(node);
 *   wc.driveForward(0.1, 1.0);        // all wheels forward
 *   wc.driveTurn(0.1, 1.0);           // differential turn
 *
 * Linear speed (m/s) is converted internally to angular velocity (rad/s)
 * using the wheel radius (0.04 m).
 */
class WheelCommander
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  /** Wheel radius from far_common_properties.xml.xacro */
  static constexpr double WHEEL_RADIUS = 0.04;  // meters

  /**
   * @param node  A fully initialised rclcpp node (use_sim_time and other
   *              parameters should be set on it before construction).
   * @param action_topic  Action server topic, defaults to
   *                      "/wheel_controller/follow_joint_trajectory".
   */
  explicit WheelCommander(
    rclcpp::Node::SharedPtr node,
    const std::string & action_topic = "/wheel_controller/follow_joint_trajectory");

  virtual ~WheelCommander() = default;

  // -- wait helpers -------------------------------------------------------

  /**
   * @brief Block until all four wheel joint positions are received on
   *        /joint_states (or until timeout).
   * @return true on success, false on timeout.
   */
  bool waitForJointStates(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  /**
   * @brief Block until the wheel controller action server is available.
   * @return true on success, false on timeout.
   */
  bool waitForActionServer(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  // -- drive commands -----------------------------------------------------

  /**
   * @brief All 4 wheels rotate forward at the same linear speed.
   * @param linear_speed  Desired ground speed (m/s), default 0.1.
   * @param duration      Movement duration (seconds), default 1.0.
   * @return true if the trajectory completed successfully.
   */
  bool driveTurn(double linear_speed = 0.1, double duration = 1.0);

  /**
   * @brief Differential steering: wheel_joint_1,2 forward,
   *        wheel_joint_3,4 reverse.
   * @param linear_speed  Base linear speed (m/s), default 0.1.
   * @param duration      Movement duration (seconds), default 1.0.
   * @return true if the trajectory completed successfully.
   */
  bool driveForward(double linear_speed = 0.1, double duration = 1.0);

  // -- low-level API (for advanced use) -----------------------------------

  /**
   * @brief Send a custom velocity profile.
   * @param velocities  4-element vector in controller joint order:
   *                    [wheel_joint_4, wheel_joint_3, wheel_joint_2, wheel_joint_1].
   * @param duration    Movement duration (seconds).
   * @return true if the trajectory completed successfully.
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
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::map<std::string, double> current_positions_;
  std::string action_topic_;
};

}  // namespace robot_commander

#endif  // ROBOT_COMMANDER__WHEEL_COMMANDER_HPP_
