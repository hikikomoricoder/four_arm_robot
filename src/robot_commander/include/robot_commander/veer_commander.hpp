#ifndef ROBOT_COMMANDER__VEER_COMMANDER_HPP_
#define ROBOT_COMMANDER__VEER_COMMANDER_HPP_

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
 * @brief Sends position-based trajectory commands to the veer_controller
 *        (joint_trajectory_controller) via its FollowJointTrajectory action.
 *
 * The veer_controller manages arm_veer_joint_{1..4} which rotate the
 * wheel-steering veer links around the Z axis.  Each joint has limits
 * [-pi, pi].
 *
 * At the home (initial) position every arm_veer_joint is at 0 rad
 * (URDF zero).  In real-space TFs the veer links appear rotated 90°
 * due to the kinematic chain geometry.
 *
 * Usage:
 *   auto node = std::make_shared<rclcpp::Node>(...);
 *   VeerCommander vc(node);
 *   vc.goHome();             // all joints to 0
 *   vc.setForwardState();    // forward/backward configuration
 */
class VeerCommander
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  /**
   * @param node  A fully initialised rclcpp node (use_sim_time and other
   *              parameters should be set on it before construction).
   * @param action_topic  Action server topic, defaults to
   *                      "/veer_controller/follow_joint_trajectory".
   */
  explicit VeerCommander(
    rclcpp::Node::SharedPtr node,
    const std::string & action_topic = "/veer_controller/follow_joint_trajectory");

  virtual ~VeerCommander() = default;

  // -- wait helpers -------------------------------------------------------

  /**
   * @brief Block until all four veer joint positions are received on
   *        /joint_states (or until timeout).
   * @return true on success, false on timeout.
   */
  bool waitForJointStates(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  /**
   * @brief Block until the veer controller action server is available.
   * @return true on success, false on timeout.
   */
  bool waitForActionServer(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  // -- veer commands ------------------------------------------------------

  /**
   * @brief Move all four veer joints to the home position (0 rad).
   *
   * Home corresponds to the URDF zero configuration.  In real-space TFs
   * each veer link appears rotated 90° due to the kinematic chain.
   *
   * @param duration  Movement duration (seconds), default 3.0.
   * @return true if the trajectory completed successfully.
   */
  bool goHome(double duration = 3.0);

  /**
   * @brief Switch to the forward/backward state.
   *
   * Relative to the current position:
   *   - arm_veer_joint_1  stays (no change)
   *   - arm_veer_joint_2  rotates +pi/2  (+90°)
   *   - arm_veer_joint_3  stays (no change)
   *   - arm_veer_joint_4  rotates +pi/2  (+90°)
   *
   * In the resulting state, joints 1&2 have the same TF direction,
   * joints 3&4 have the same TF direction, and the two groups are
   * 180° apart — analogous to wheel_commander's driveForward where
   * wheels 1,2 rotate together and wheels 3,4 rotate together but
   * opposite to 1,2.
   *
   * @param duration  Movement duration (seconds), default 3.0.
   * @return true if the trajectory completed successfully.
   */
  bool setForwardState(double duration = 3.0);

  /**
   * @brief Turn state: all four veer joints to +45° (pi/4 rad).
   *
   * This is an absolute target — every joint goes to pi/4 rad.
   *
   * @param duration  Movement duration (seconds), default 3.0.
   * @return true if the trajectory completed successfully.
   */
  bool setTurnState(double duration = 3.0);

  // -- low-level API (for advanced use) -----------------------------------

  /**
   * @brief Send a position goal for all four veer joints.
   * @param positions  4-element vector in controller joint order:
   *                   [arm_veer_joint_4, arm_veer_joint_3,
   *                    arm_veer_joint_2, arm_veer_joint_1].
   * @param duration   Movement duration (seconds).
   * @return true if the trajectory completed successfully.
   */
  bool sendPositionGoal(const std::vector<double> & positions, double duration);

  /** Controller joint order (read-only) */
  static const std::vector<std::string> & jointNames()
  {
    static const std::vector<std::string> kNames{
      "arm_veer_joint_4", "arm_veer_joint_3",
      "arm_veer_joint_2", "arm_veer_joint_1"};
    return kNames;
  }

  /** Home position (rad) — URDF zero */
  static const std::vector<double> & homePositions()
  {
    static const std::vector<double> kHome{0.0, 0.0, 0.0, 0.0};
    return kHome;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::map<std::string, double> current_positions_;
  std::string action_topic_;
};

}  // namespace robot_commander

#endif  // ROBOT_COMMANDER__VEER_COMMANDER_HPP_
