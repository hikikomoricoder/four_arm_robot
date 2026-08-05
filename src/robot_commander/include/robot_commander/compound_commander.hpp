#ifndef ROBOT_COMMANDER__COMPOUND_COMMANDER_HPP_
#define ROBOT_COMMANDER__COMPOUND_COMMANDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/srv/group_state_manager.hpp>
#include <robot_commander/arm_commander.hpp>
#include <robot_commander/wheel_commander.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace robot_commander
{

/**
 * @brief Compound (arm + mobile_base) preset poses.
 *
 * Reimplements the arm preset poses (home, low, high, rhombus_1, rhombus_2)
 * so that every preset also drives the mobile_base deformation: while the
 * arms move, the wheels run the lift-mode half-sine velocity profile for
 * exactly the same total duration (the veer joints are expected to already
 * be in the lift configuration, -45°).
 *
 * Every preset forms a control loop with the /group_state_manager service:
 *
 *   1. Pre-check (get_all) — the preset is only executed when:
 *        - arm   status   == "free"
 *        - veer  position == "lift"
 *        - veer  status   == "free"
 *        - wheel status   == "free"
 *      Otherwise the command is rejected and nothing moves.
 *   2. Reserve (set_group):
 *        - arm   position = target preset, status = "occupy"
 *        - veer  status   = "occupy"   (position stays "lift")
 *        - wheel position = "lift",    status = "occupy"
 *   3. Execute — arm trajectory steps (blocking action goals) run
 *      concurrently with the wheel lift profile in a background thread;
 *      both span the same total duration T:
 *        wheel w(t) = (kLiftPeakLinearSpeed / WHEEL_RADIUS) * sin(pi * t / T)
 *      published at 50 Hz (sim time), t in [0, T].
 *   4. Release (set_group) — arm / veer / wheel status back to "free"
 *      (positions unchanged).
 *
 * Usage:
 *   auto node = std::make_shared<rclcpp::Node>(...);
 *   CompoundCommander cc(node);
 *   cc.waitForJointStates();
 *   cc.waitForActionServer();
 *   cc.setHomeState();      // arms to 0 + wheel lift profile
 *   cc.setLowState();       // low preset + wheel lift profile
 *   cc.setHighState();      // high preset + wheel lift profile
 *   cc.setRhombus1State();  // rhombus 1 + wheel lift profile
 *   cc.setRhombus2State();  // rhombus 2 + wheel lift profile
 */
class CompoundCommander
{
public:
  /**
   * @param node  A fully initialised rclcpp node (use_sim_time and other
   *              parameters should be set on it before construction).
   */
  explicit CompoundCommander(rclcpp::Node::SharedPtr node);

  virtual ~CompoundCommander() = default;

  // -- wait helpers -------------------------------------------------------

  /**
   * @brief Block until all 16 arm joint and 4 wheel joint positions are
   *        received on /joint_states (or until timeout, per group).
   * @return true on success, false on timeout.
   */
  bool waitForJointStates(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  /**
   * @brief Block until the all_arms_controller action server is available.
   * @return true on success, false on timeout.
   */
  bool waitForActionServer(const std::chrono::seconds & timeout = std::chrono::seconds(5));

  // -- compound preset commands -------------------------------------------

  /**
   * @brief "Home" preset — all 16 arm joints back to 0 rad (single step),
   *        with the wheel lift profile running for the same duration.
   * @param duration  Movement duration (seconds), default 3.0.
   * @return true if the compound motion completed successfully.
   */
  bool setHomeState(double duration = 3.0);

  /**
   * @brief "Low" preset — all four arms: j3 -pi/4, j5 +pi/2, j7 -pi/4
   *        (two-step: home first, then offset), with the wheel lift
   *        profile spanning both steps.
   * @param duration  Desired duration of the offset step (seconds),
   *                  default 3.0 (at least 2.0 s is enforced).
   * @return true if the compound motion completed successfully.
   */
  bool setLowState(double duration = 3.0);

  /**
   * @brief "High" preset — all four arms: j3 +pi/8, j5 -pi/4, j7 +pi/8
   *        (two-step: home first, then offset), with the wheel lift
   *        profile spanning both steps.
   * @param duration  Desired duration of the offset step (seconds),
   *                  default 3.0 (at least 2.0 s is enforced).
   * @return true if the compound motion completed successfully.
   */
  bool setHighState(double duration = 3.0);

  /**
   * @brief "Rhombus 1" preset — j1 = +pi/4, -pi/4, +pi/4, -pi/4
   *        (two-step: home first, then offset), with the wheel lift
   *        profile spanning both steps.
   * @param duration  Desired duration of the offset step (seconds),
   *                  default 3.0 (at least 2.0 s is enforced).
   * @return true if the compound motion completed successfully.
   */
  bool setRhombus1State(double duration = 3.0);

  /**
   * @brief "Rhombus 2" preset — j1 = -pi/4, +pi/4, -pi/4, +pi/4
   *        (two-step: home first, then offset), with the wheel lift
   *        profile spanning both steps.
   * @param duration  Desired duration of the offset step (seconds),
   *                  default 3.0 (at least 2.0 s is enforced).
   * @return true if the compound motion completed successfully.
   */
  bool setRhombus2State(double duration = 3.0);

private:
  /** One arm trajectory step: target positions + step duration. */
  struct ArmStep
  {
    std::vector<double> targets;  ///< 16-element vector, controller joint order
    double duration;              ///< step duration (s)
  };

  // -- state management helpers ------------------------------------------

  /**
   * @brief Pre-check via /group_state_manager (get_all): arm status "free",
   *        veer position "lift", veer status "free", wheel status "free".
   * @return true if a compound preset may execute.
   */
  bool checkPreconditions();

  /**
   * @brief Reserve the arm / veer / wheel groups for a preset:
   *        arm position=`arm_mode` + status "occupy", veer status "occupy"
   *        (position unchanged), wheel position "lift" + status "occupy".
   *        Rolls back already-reserved groups on partial failure.
   * @param arm_mode  Target arm preset name ("home", "low", "high",
   *                  "rhombus_1", "rhombus_2").
   * @return true if all three groups were reserved.
   */
  bool reserveGroups(const std::string & arm_mode);

  /**
   * @brief Release the arm / veer / wheel groups back to "free"
   *        (positions unchanged, best effort).
   */
  void releaseGroups();

  /**
   * @brief Call set_group on /group_state_manager once.
   * @return true if the service call succeeded and returned success.
   */
  bool setGroup(
    const std::string & group, const std::string & position,
    const std::string & status);

  // -- motion helpers ------------------------------------------------------

  /**
   * @brief Build the two-step (home, then offset) arm step sequence used
   *        by the offset presets.  The home-step duration is derived from
   *        the current displacement and the URDF velocity limit; the
   *        offset-step duration is at least kMinOffsetDuration.
   * @return false if no joint state data is available yet.
   */
  bool buildOffsetSteps(
    const std::vector<double> & offsets, double duration,
    std::vector<ArmStep> & steps);

  /**
   * @brief Run the arm steps while concurrently driving the wheel lift
   *        half-sine profile, both using the same planned duration.
   *
   * Two independent commands start at the same time:
   *   - Arm steps run on the main thread (blocking sendPositionGoal,
   *     sim-time based).
   *   - Wheel lift profile runs in a background thread using wall-clock
   *     timing (std::chrono::steady_clock), completely independent of
   *     sim time and arm state.
   *
   * Because Gazebo executes trajectories ~4x slower than requested, the
   * wheel half-sine profile spans total_duration * kSimDurationScale:
   *   w(t) = (peak_linear_speed / WHEEL_RADIUS) * sin(pi * t / T)
   * for t in [0, T], then zero — so both commands finish together.
   *
   * @return true if every arm step succeeded.
   */
  bool runWithWheelLift(
    const std::vector<ArmStep> & steps, double peak_linear_speed);

  rclcpp::Node::SharedPtr node_;
  ArmCommander arm_commander_;
  WheelCommander wheel_commander_;
  rclcpp::Client<robot_interfaces::srv::GroupStateManager>::SharedPtr state_manager_client_;
};

}  // namespace robot_commander

#endif  // ROBOT_COMMANDER__COMPOUND_COMMANDER_HPP_
