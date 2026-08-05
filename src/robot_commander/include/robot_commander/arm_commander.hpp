#ifndef ROBOT_COMMANDER__ARM_COMMANDER_HPP_
#define ROBOT_COMMANDER__ARM_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <future>
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
 *
 * This class is the low-level arm-motion layer: it only knows how to send
 * trajectories to the arm controller.  The named preset poses (home, low,
 * high, rhombus_1, rhombus_2) — which additionally coordinate the
 * mobile_base (wheel lift mode) and the /group_state_manager state lock —
 * live in CompoundCommander.
 *
 * Usage:
 *   auto node = std::make_shared<rclcpp::Node>(...);
 *   ArmCommander ac(node);
 *   ac.waitForJointStates();
 *   ac.waitForActionServer();
 *   ac.sendPositionGoal(ArmCommander::homePositions(), 3.0);  // all joints to 0
 */
class ArmCommander
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using SendGoalFuture = std::shared_future<GoalHandle::SharedPtr>;
  using WrappedResult = rclcpp_action::Client<FollowJointTrajectory>::WrappedResult;
  using ResultFuture = std::shared_future<WrappedResult>;

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

  // -- low-level API ------------------------------------------------------

  /**
   * @brief Send a position goal for all 16 arm joints and block until the
   *        trajectory finishes.
   * @param positions  16-element vector in controller joint order
   *                   (see jointNames()).
   * @param duration   Movement duration (seconds).
   * @return true if the trajectory completed successfully.
   */
  bool sendPositionGoal(const std::vector<double> & positions, double duration);

  /**
   * @brief Validate and send a position goal for all 16 arm joints without
   *        blocking.
   *
   * The returned future becomes ready once the controller answers the goal
   * request; the caller is responsible for spinning the node in the
   * meantime (e.g. via rclcpp::spin_some) and for waiting on the result
   * with asyncGetResult().  This allows CompoundCommander to drive the
   * wheel lift profile while the arm trajectory executes.
   *
   * @param positions  16-element vector in controller joint order.
   * @param duration   Movement duration (seconds).
   * @param goal_future  Output: the goal-handle future (only set when true
   *                     is returned).
   * @return true if the goal was sent, false on validation errors.
   */
  bool asyncSendPositionGoal(
    const std::vector<double> & positions, double duration,
    SendGoalFuture & goal_future);

  /**
   * @brief Request the result of an accepted goal without blocking.
   * @param goal_handle  Goal handle obtained from the SendGoalFuture.
   * @return Future for the wrapped result (poll it while spinning the node).
   */
  ResultFuture asyncGetResult(const GoalHandle::SharedPtr & goal_handle);

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

  /** Latest positions received on /joint_states, keyed by joint name */
  const std::map<std::string, double> & currentPositions() const
  {
    return current_positions_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::map<std::string, double> current_positions_;
  std::string action_topic_;
};

}  // namespace robot_commander

#endif  // ROBOT_COMMANDER__ARM_COMMANDER_HPP_
