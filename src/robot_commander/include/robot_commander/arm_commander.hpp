#ifndef ROBOT_COMMANDER__ARM_COMMANDER_HPP_
#define ROBOT_COMMANDER__ARM_COMMANDER_HPP_

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
 * @brief Sends position-based trajectory commands to the all_arms_controller
 *        (joint_trajectory_controller) via its FollowJointTrajectory action.
 *
 * The all_arms_controller manages all 16 arm joints of the four (identical)
 * arms in the order:
 *
 *   [arm_joint_1_1, arm_joint_3_1, arm_joint_5_1, arm_joint_7_1,   // arm 1
 *    arm_joint_1_2, arm_joint_3_2, arm_joint_5_2, arm_joint_7_2,   // arm 2
 *    arm_joint_1_3, arm_joint_3_3, arm_joint_5_3, arm_joint_7_3,   // arm 3
 *    arm_joint_1_4, arm_joint_3_4, arm_joint_5_4, arm_joint_7_4]   // arm 4
 *
 * `arm_joint_7_x` is a belt-driven mimic of `arm_joint_5_x`:
 * arm_joint_5_x rotating N deg makes arm_joint_7_x rotate -N/2 deg.
 * Gazebo does not support mimic joints, so the two joints are controlled
 * independently while every preset keeps the 1/2 reverse relationship.
 *
 * At the home (initial) position every arm joint is at 0 rad (URDF zero).
 * All preset poses are defined as offsets relative to this initial state,
 * so they first return home and then apply the offset (two-step motion).
 *
 * Usage:
 *   auto node = std::make_shared<rclcpp::Node>(...);
 *   ArmCommander ac(node);
 *   ac.setHomeState();      // all 16 arm joints to 0
 *   ac.setLowState();       // arms down: j3 -45°, j5 +90°, j7 -45°
 *   ac.setHighState();      // arms up:   j3 +22.5°, j5 -45°, j7 +22.5°
 *   ac.setRhombus1State();  // diamond: j1 = +45°, -45°, +45°, -45°
 *   ac.setRhombus2State();  // diamond: j1 = -45°, +45°, -45°, +45°
 */
class ArmCommander
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  /**
   * @param node  A fully initialised rclcpp node (use_sim_time and other
   *              parameters should be set on it before construction).
   * @param action_topic  Action server topic, defaults to
   *                      "/all_arms_controller/follow_joint_trajectory".
   */
  explicit ArmCommander(
    rclcpp::Node::SharedPtr node,
    const std::string & action_topic = "/all_arms_controller/follow_joint_trajectory");

  virtual ~ArmCommander() = default;

  // -- wait helpers -------------------------------------------------------

  /**
   * @brief Block until all 16 arm joint positions are received on
   *        /joint_states (or until timeout).
   * @return true on success, false on timeout.
   */
  bool waitForJointStates(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  /**
   * @brief Block until the all_arms_controller action server is available.
   * @return true on success, false on timeout.
   */
  bool waitForActionServer(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  // -- arm preset commands -------------------------------------------------

  /**
   * @brief Move all 16 arm joints back to the home (initial) position
   *        (0 rad, URDF zero).
   * @param duration  Movement duration (seconds), default 3.0.
   * @return true if the trajectory completed successfully.
   */
  bool setHomeState(double duration = 3.0);

  /**
   * @brief "Low" preset — all four arms perform the same motion
   *        relative to the initial state:
   *          - arm_joint_3_x  -pi/4
   *          - arm_joint_5_x  +pi/2
   *          - arm_joint_7_x  -pi/4   (mimic of j5: -1/2 * (+pi/2))
   *        arm_joint_1_x stays at 0.
   *
   * Two-step motion: 1) setHomeState (auto duration), 2) apply the offset
   * (at least 2.0 s to converge reliably under Gazebo physics).
   *
   * @param duration  Desired movement duration for the offset step
   *                  (seconds), default 3.0.
   * @return true if the trajectory completed successfully.
   */
  bool setLowState(double duration = 3.0);

  /**
   * @brief "High" preset — all four arms perform the same motion
   *        relative to the initial state:
   *          - arm_joint_3_x  +pi/8
   *          - arm_joint_5_x  -pi/4
   *          - arm_joint_7_x  +pi/8   (mimic of j5: -1/2 * (-pi/4))
   *        arm_joint_1_x stays at 0.
   *
   * Two-step motion: 1) setHomeState (auto duration), 2) apply the offset
   * (at least 2.0 s to converge reliably under Gazebo physics).
   *
   * @param duration  Desired movement duration for the offset step
   *                  (seconds), default 3.0.
   * @return true if the trajectory completed successfully.
   */
  bool setHighState(double duration = 3.0);

  /**
   * @brief "Rhombus 1" preset — only arm_joint_1_x moves, forming a
   *        diamond pattern relative to the initial state:
   *          - arm_joint_1_1  +pi/4
   *          - arm_joint_1_2  -pi/4
   *          - arm_joint_1_3  +pi/4
   *          - arm_joint_1_4  -pi/4
   *        All other arm joints stay at 0.
   *
   * Two-step motion: 1) setHomeState (auto duration), 2) apply the offset
   * (at least 2.0 s to converge reliably under Gazebo physics).
   *
   * @param duration  Desired movement duration for the offset step
   *                  (seconds), default 3.0.
   * @return true if the trajectory completed successfully.
   */
  bool setRhombus1State(double duration = 3.0);

  /**
   * @brief "Rhombus 2" preset — only arm_joint_1_x moves, forming the
   *        opposite diamond pattern relative to the initial state:
   *          - arm_joint_1_1  -pi/4
   *          - arm_joint_1_2  +pi/4
   *          - arm_joint_1_3  -pi/4
   *          - arm_joint_1_4  +pi/4
   *        All other arm joints stay at 0.
   *
   * Two-step motion: 1) setHomeState (auto duration), 2) apply the offset
   * (at least 2.0 s to converge reliably under Gazebo physics).
   *
   * @param duration  Desired movement duration for the offset step
   *                  (seconds), default 3.0.
   * @return true if the trajectory completed successfully.
   */
  bool setRhombus2State(double duration = 3.0);

  // -- low-level API (for advanced use) -----------------------------------

  /**
   * @brief Send a position goal for all 16 arm joints.
   * @param positions  16-element vector in controller joint order
   *                   (see jointNames()).
   * @param duration   Movement duration (seconds).
   * @return true if the trajectory completed successfully.
   */
  bool sendPositionGoal(const std::vector<double> & positions, double duration);

  /** Controller joint order (read-only) */
  static const std::vector<std::string> & jointNames()
  {
    static const std::vector<std::string> kNames{
      "arm_joint_1_1", "arm_joint_3_1", "arm_joint_5_1", "arm_joint_7_1",
      "arm_joint_1_2", "arm_joint_3_2", "arm_joint_5_2", "arm_joint_7_2",
      "arm_joint_1_3", "arm_joint_3_3", "arm_joint_5_3", "arm_joint_7_3",
      "arm_joint_1_4", "arm_joint_3_4", "arm_joint_5_4", "arm_joint_7_4"};
    return kNames;
  }

  /** Home position (rad) — URDF zero */
  static const std::vector<double> & homePositions()
  {
    static const std::vector<double> kHome(16, 0.0);
    return kHome;
  }

private:
  /**
   * @brief Two-step motion helper: return home first (duration computed
   *        from the URDF velocity limit and the current displacement),
   *        then move to `offsets` relative to the home position.
   * @param offsets   16-element offset vector in controller joint order.
   * @param duration  Desired movement duration for the offset step.
   * @return true if both trajectories completed successfully.
   */
  bool homeThenOffset(const std::vector<double> & offsets, double duration);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::map<std::string, double> current_positions_;
  std::string action_topic_;
};

}  // namespace robot_commander

#endif  // ROBOT_COMMANDER__ARM_COMMANDER_HPP_
