#include <robot_commander/compound_commander.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace robot_commander
{

namespace
{
// Max joint velocity from joint_limits.yaml (1.0 rad/s for all arm joints)
constexpr double kMaxJointVelocity = 1.0;  // rad/s

// Minimum duration of the offset step, tuned for reliable convergence
// under Gazebo physics (same as the veer forward preset).
constexpr double kMinOffsetDuration = 2.0;  // s

// If every arm joint is already within this distance of home, the home step
// is skipped and the preset is sent as a single arm goal (the arm is then
// commanded exactly once).
constexpr double kHomeEpsilon = 0.01;  // rad

// ============================================================================
//  Wheel lift geometry — mirrors the URDF (far_common_properties.xml.xacro,
//  main_arm_units.xml.xacro).  See commander.md section 4 "wheel lift 运动
//  距离与速度曲线" for the full derivation.
// ============================================================================

// -- URDF dimensions (m) ---------------------------------------------------
constexpr double kArmBaseLength = 0.10;
constexpr double kArmBaseHeight = 0.03;
constexpr double kArmJoint1Length = 0.04;
constexpr double kArmJoint1Radius = 0.03;
constexpr double kArmJoint2Radius = 0.03;
constexpr double kArmJoint3Radius = 0.03;
constexpr double kArmLink1Height = 0.01;
constexpr double kArmLink2Height = 0.25;
constexpr double kArmLink3Height = 0.25;

// -- Chain segments of one arm (J1 -> next arm base), at arm_joint_1 = 0 ----
// J1 -> J3 vertical offset: arm_joint_1_length + arm_link_1_height +
// arm_joint_2_radius
constexpr double kZJ1toJ3 = kArmJoint1Length + kArmLink1Height + kArmJoint2Radius;
// J3 -> J5 link length: arm_link_2_height + arm_joint_2_radius +
// arm_joint_3_radius
constexpr double kLenJ3toJ5 = kArmLink2Height + kArmJoint2Radius + kArmJoint3Radius;
// J5 -> J7 link length: arm_link_3_height + arm_joint_3_radius +
// arm_joint_1_radius
constexpr double kLenJ5toJ7 = kArmLink3Height + kArmJoint3Radius + kArmJoint1Radius;
// J7 -> next base (group_joint) offset: (arm_base_length/2, 0,
// arm_joint_2_radius + arm_link_1_height + arm_joint_1_length +
// arm_base_height/2)
constexpr double kGroupOffsetX = kArmBaseLength / 2.0;
constexpr double kGroupOffsetZ =
  kArmJoint2Radius + kArmLink1Height + kArmJoint1Length + kArmBaseHeight / 2.0;

// Horizontal separation of two consecutive arm bases.  All presets keep the
// total arm pitch at -pi, so the group offset is applied upside down
// (-kGroupOffsetX, 0, -kGroupOffsetZ) and the bases stay coplanar:
//   L(q3, q5) = kGroupOffsetX + kLenJ3toJ5 * sin(pi/4 - q3)
//                           + kLenJ5toJ7 * sin(3*pi/4 - q3 - q5)
constexpr double baseSeparation(double q3, double q5)
{
  return kGroupOffsetX +
         kLenJ3toJ5 * std::sin(M_PI_4 - q3) +
         kLenJ5toJ7 * std::sin(3.0 * M_PI_4 - q3 - q5);
}

constexpr double kBaseSepHome = baseSeparation(0.0, 0.0);              // ~0.4884 m
constexpr double kBaseSepLow = baseSeparation(-M_PI_4, M_PI_2);        // 0.67 m
constexpr double kBaseSepHigh = baseSeparation(M_PI / 8.0, -M_PI_4);   // ~0.2873 m
}  // namespace

// ============================================================================
//  Constructor
// ============================================================================

CompoundCommander::CompoundCommander(rclcpp::Node::SharedPtr node)
: node_(std::move(node)),
  arm_commander_(node_),
  wheel_commander_(node_)
{
  // Create the group_state_manager service client
  state_manager_client_ =
    node_->create_client<robot_interfaces::srv::GroupStateManager>("group_state_manager");

  // Signed wheel lift travel per preset: the change of the adjacent arm-base
  // separation vs home (see commander.md).  Positive = bases spread apart
  // (home -> low), negative = bases contract (home -> high).  The rhombus
  // presets do not change the base separation, so they stay 0.  The return
  // trip (preset -> home) uses the NEGATED entry of this table, keyed by
  // the state-manager arm position (see setHomeState()).
  preset_lift_distance_ = {
    {"home", 0.0},
    {"low", kBaseSepLow - kBaseSepHome},
    {"high", kBaseSepHigh - kBaseSepHome},
    {"rhombus_1", 0.0},
    {"rhombus_2", 0.0}};

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] Wheel lift geometry (URDF): base separation "
              "home %.4f m, low %.4f m, high %.4f m (travel vs home: low %+.4f m, "
              "high %+.4f m); bases coplanar (net dz = %.4f m/arm)",
              kBaseSepHome, kBaseSepLow, kBaseSepHigh,
              kBaseSepLow - kBaseSepHome, kBaseSepHigh - kBaseSepHome,
              kZJ1toJ3 - kGroupOffsetZ);
}

// ============================================================================
//  presetLiftDistance
// ============================================================================

double CompoundCommander::presetLiftDistance(const std::string & preset) const
{
  const auto it = preset_lift_distance_.find(preset);
  return it != preset_lift_distance_.end() ? it->second : 0.0;
}

// ============================================================================
//  waitForJointStates
// ============================================================================

bool CompoundCommander::waitForJointStates(const std::chrono::seconds & timeout)
{
  return arm_commander_.waitForJointStates(timeout) &&
         wheel_commander_.waitForJointStates(timeout);
}

// ============================================================================
//  waitForActionServer
// ============================================================================

bool CompoundCommander::waitForActionServer(const std::chrono::seconds & timeout)
{
  return arm_commander_.waitForActionServer(timeout);
}

// ============================================================================
//  setHomeState
// ============================================================================

bool CompoundCommander::setHomeState(double duration)
{
  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] setHomeState: all 16 arm joints to 0 rad "
              "over %.1f s + wheel lift profile",
              duration);

  const std::vector<ArmStep> steps{{ArmCommander::homePositions(), duration}};

  // Pre-check and fetch the current arm position: the wheel scheme for the
  // trip back home is a TABLE LOOKUP keyed by it (no joint-state
  // measurement).  Only low -> home is fully tested; the other entries are
  // provided for completeness.
  const std::string arm_position = queryArmPosition();
  if (arm_position.empty()) {
    return false;
  }

  double lift_distance = 0.0;
  const auto it = preset_lift_distance_.find(arm_position);
  if (it != preset_lift_distance_.end()) {
    lift_distance = -it->second;  // reverse of the home -> preset trip
  }
  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] setHomeState: arm position '%s' -> wheel "
              "lift travel %+.4f m (table lookup)",
              arm_position.c_str(), lift_distance);

  if (!reserveGroups("home")) {
    return false;
  }

  const bool ok = runWithWheelLift(steps, lift_distance);
  releaseGroups();
  return ok;
}

// ============================================================================
//  setLowState
// ============================================================================

bool CompoundCommander::setLowState(double duration)
{
  // Per-arm block [j1, j3, j5, j7]; all four arms move identically:
  //   j3 -pi/4, j5 +pi/2, j7 -pi/4 (mimic of j5: -1/2 * (+pi/2))
  std::vector<double> offsets(16, 0.0);
  for (int arm = 0; arm < 4; ++arm) {
    const size_t base = static_cast<size_t>(arm) * 4;
    offsets[base + 1] = -M_PI_4;   // arm_joint_3_x
    offsets[base + 2] = M_PI_2;    // arm_joint_5_x
    offsets[base + 3] = -M_PI_4;   // arm_joint_7_x
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] setLowState: j3 -pi/4, j5 +pi/2, j7 -pi/4 "
              "for all arms + wheel lift profile");

  std::vector<ArmStep> steps;
  if (!buildOffsetSteps(offsets, duration, steps)) {
    return false;
  }
  if (!checkPreconditions()) {
    return false;
  }
  if (!reserveGroups("low")) {
    return false;
  }

  const bool ok = runWithWheelLift(steps, preset_lift_distance_.at("low"));
  releaseGroups();
  return ok;
}

// ============================================================================
//  setHighState
// ============================================================================

bool CompoundCommander::setHighState(double duration)
{
  // Per-arm block [j1, j3, j5, j7]; all four arms move identically:
  //   j3 +pi/8, j5 -pi/4, j7 +pi/8 (mimic of j5: -1/2 * (-pi/4))
  std::vector<double> offsets(16, 0.0);
  for (int arm = 0; arm < 4; ++arm) {
    const size_t base = static_cast<size_t>(arm) * 4;
    offsets[base + 1] = M_PI / 8.0;   // arm_joint_3_x
    offsets[base + 2] = -M_PI_4;      // arm_joint_5_x
    offsets[base + 3] = M_PI / 8.0;   // arm_joint_7_x
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] setHighState: j3 +pi/8, j5 -pi/4, j7 +pi/8 "
              "for all arms + wheel lift profile");

  std::vector<ArmStep> steps;
  if (!buildOffsetSteps(offsets, duration, steps)) {
    return false;
  }
  if (!checkPreconditions()) {
    return false;
  }
  if (!reserveGroups("high")) {
    return false;
  }

  const bool ok = runWithWheelLift(steps, preset_lift_distance_.at("high"));
  releaseGroups();
  return ok;
}

// ============================================================================
//  setRhombus1State
// ============================================================================

bool CompoundCommander::setRhombus1State(double duration)
{
  // Only arm_joint_1_x moves, forming a diamond pattern:
  //   j1_1 +pi/4, j1_2 -pi/4, j1_3 +pi/4, j1_4 -pi/4
  std::vector<double> offsets(16, 0.0);
  offsets[0]  = M_PI_4;   // arm_joint_1_1
  offsets[4]  = -M_PI_4;  // arm_joint_1_2
  offsets[8]  = M_PI_4;   // arm_joint_1_3
  offsets[12] = -M_PI_4;  // arm_joint_1_4

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] setRhombus1State: j1 = +pi/4, -pi/4, +pi/4, "
              "-pi/4 + wheel lift profile");

  std::vector<ArmStep> steps;
  if (!buildOffsetSteps(offsets, duration, steps)) {
    return false;
  }
  if (!checkPreconditions()) {
    return false;
  }
  if (!reserveGroups("rhombus_1")) {
    return false;
  }

  const bool ok = runWithWheelLift(steps, preset_lift_distance_.at("rhombus_1"));
  releaseGroups();
  return ok;
}

// ============================================================================
//  setRhombus2State
// ============================================================================

bool CompoundCommander::setRhombus2State(double duration)
{
  // Only arm_joint_1_x moves, opposite diamond pattern:
  //   j1_1 -pi/4, j1_2 +pi/4, j1_3 -pi/4, j1_4 +pi/4
  std::vector<double> offsets(16, 0.0);
  offsets[0]  = -M_PI_4;  // arm_joint_1_1
  offsets[4]  = M_PI_4;   // arm_joint_1_2
  offsets[8]  = -M_PI_4;  // arm_joint_1_3
  offsets[12] = M_PI_4;   // arm_joint_1_4

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] setRhombus2State: j1 = -pi/4, +pi/4, -pi/4, "
              "+pi/4 + wheel lift profile");

  std::vector<ArmStep> steps;
  if (!buildOffsetSteps(offsets, duration, steps)) {
    return false;
  }
  if (!checkPreconditions()) {
    return false;
  }
  if (!reserveGroups("rhombus_2")) {
    return false;
  }

  const bool ok = runWithWheelLift(steps, preset_lift_distance_.at("rhombus_2"));
  releaseGroups();
  return ok;
}

// ============================================================================
//  queryArmPosition   (state management helper)
// ============================================================================

std::string CompoundCommander::queryArmPosition()
{
  if (!state_manager_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[CompoundCommander] group_state_manager service not available");
    return "";
  }

  auto req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  req->command = "get_all";

  auto future = state_manager_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[CompoundCommander] Failed to call group_state_manager (get_all)");
    return "";
  }

  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[CompoundCommander] get_all failed: %s", resp->message.c_str());
    return "";
  }

  // Common preconditions for every compound preset: all groups must be free
  // and the veer must already be in the lift configuration.  The arm
  // position itself is NOT constrained here (setHomeState needs it to
  // select the wheel scheme; the offset presets check it separately).
  if (resp->arm_status != "free" ||
      resp->veer_position != "lift" ||
      resp->veer_status != "free" ||
      resp->wheel_status != "free")
  {
    RCLCPP_WARN(node_->get_logger(),
                "[CompoundCommander] Preconditions not met, cannot execute: "
                "arm status '%s' (need 'free'), veer position '%s' (need 'lift'), "
                "veer status '%s' (need 'free'), wheel status '%s' (need 'free')",
                resp->arm_status.c_str(), resp->veer_position.c_str(),
                resp->veer_status.c_str(), resp->wheel_status.c_str());
    return "";
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] Preconditions OK: arm '%s'+free, "
              "veer lift+free, wheel free",
              resp->arm_position.c_str());
  return resp->arm_position;
}

// ============================================================================
//  checkPreconditions   (state management helper)
// ============================================================================

bool CompoundCommander::checkPreconditions()
{
  const std::string arm_position = queryArmPosition();
  if (arm_position.empty()) {
    return false;
  }

  // The offset presets always depart FROM home (their targets are defined
  // relative to it), so the arm must currently be at home.
  if (arm_position != "home") {
    RCLCPP_WARN(node_->get_logger(),
                "[CompoundCommander] Preconditions not met, cannot execute: "
                "arm position '%s' (need 'home')", arm_position.c_str());
    return false;
  }
  return true;
}

// ============================================================================
//  reserveGroups   (state management helper)
// ============================================================================

bool CompoundCommander::reserveGroups(const std::string & arm_mode)
{
  // arm: position = target preset, status = occupy
  if (!setGroup("arm", arm_mode, "occupy")) {
    return false;
  }
  // veer: status = occupy (position stays "lift")
  if (!setGroup("veer", "", "occupy")) {
    setGroup("arm", "", "free");  // roll back
    return false;
  }
  // wheel: position = lift, status = occupy
  if (!setGroup("wheel", "lift", "occupy")) {
    setGroup("veer", "", "free");  // roll back
    setGroup("arm", "", "free");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] Groups reserved: arm position=%s status=occupy, "
              "veer status=occupy, wheel position=lift status=occupy",
              arm_mode.c_str());
  return true;
}

// ============================================================================
//  releaseGroups   (state management helper)
// ============================================================================

void CompoundCommander::releaseGroups()
{
  // Best effort: set every status back to free, positions unchanged.
  const bool arm_ok = setGroup("arm", "", "free");
  const bool veer_ok = setGroup("veer", "", "free");
  const bool wheel_ok = setGroup("wheel", "", "free");

  if (arm_ok && veer_ok && wheel_ok) {
    RCLCPP_INFO(node_->get_logger(),
                "[CompoundCommander] Groups released: arm/veer/wheel status=free");
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "[CompoundCommander] Failed to release all groups "
                 "(arm %s, veer %s, wheel %s)",
                 arm_ok ? "ok" : "FAILED",
                 veer_ok ? "ok" : "FAILED",
                 wheel_ok ? "ok" : "FAILED");
  }
}

// ============================================================================
//  setGroup   (state management helper)
// ============================================================================

bool CompoundCommander::setGroup(
  const std::string & group, const std::string & position,
  const std::string & status)
{
  if (!state_manager_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[CompoundCommander] group_state_manager service not available");
    return false;
  }

  auto req = std::make_shared<robot_interfaces::srv::GroupStateManager::Request>();
  req->command = "set_group";
  req->group_name = group;
  req->position_name = position;  // empty = leave unchanged
  req->status_name = status;

  auto future = state_manager_client_->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[CompoundCommander] Failed to call group_state_manager (set_group %s)",
                 group.c_str());
    return false;
  }

  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[CompoundCommander] set_group %s failed: %s",
                 group.c_str(), resp->message.c_str());
    return false;
  }
  return true;
}

// ============================================================================
//  buildOffsetSteps   (motion helper)
// ============================================================================

bool CompoundCommander::buildOffsetSteps(
  const std::vector<double> & offsets, double duration,
  std::vector<ArmStep> & steps)
{
  // Ensure we have joint state data
  const auto & current = arm_commander_.currentPositions();
  if (!current.count("arm_joint_1_1")) {
    RCLCPP_WARN(node_->get_logger(),
                "[CompoundCommander] No joint state data yet — call "
                "waitForJointStates() first");
    return false;
  }

  // Step 1: return to home first, so the offset is always relative to the
  // initial (home) state.  Compute a safe duration from the joint velocity
  // limit and the current displacement.
  double max_home_delta = 0.0;
  for (const auto & jn : ArmCommander::jointNames()) {
    auto it = current.find(jn);
    if (it != current.end()) {
      max_home_delta = std::max(max_home_delta, std::abs(it->second));
    }
  }
  const double go_home_duration = std::max(max_home_delta / kMaxJointVelocity, 1.0);

  // Step 2: targets = home (0) + offsets, in controller joint order.
  // Ensure enough time for the offset movement given velocity limits and
  // Gazebo PID tracking lag (same floor as the veer forward preset).
  const double offset_duration = std::max(duration, kMinOffsetDuration);

  if (max_home_delta < kHomeEpsilon) {
    // Already at home: a single goal suffices — the arm is commanded once.
    RCLCPP_INFO(node_->get_logger(),
                "[CompoundCommander] Single-step motion (already at home, max "
                "delta %.3f rad): offsets over %.1f s",
                max_home_delta, offset_duration);
    steps = {{offsets, offset_duration}};
    return true;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] Two-step motion: home over %.1f s (max delta "
              "%.3f rad), then offsets over %.1f s",
              go_home_duration, max_home_delta, offset_duration);

  steps = {
    {ArmCommander::homePositions(), go_home_duration},
    {offsets, offset_duration}};
  return true;
}

// ============================================================================
//  currentBaseSeparation   (motion helper)
//
//  Currently unused by the presets (the home trip is a table lookup keyed
//  by the state-manager arm position); kept for a future "force return to
//  home" mode that must work regardless of the recorded state.
// ============================================================================

bool CompoundCommander::currentBaseSeparation(double & separation) const
{
  const auto & current = arm_commander_.currentPositions();
  double q3_sum = 0.0;
  double q5_sum = 0.0;
  for (int arm = 1; arm <= 4; ++arm) {
    const std::string j3 = "arm_joint_3_" + std::to_string(arm);
    const std::string j5 = "arm_joint_5_" + std::to_string(arm);
    const auto it3 = current.find(j3);
    const auto it5 = current.find(j5);
    if (it3 == current.end() || it5 == current.end()) {
      return false;
    }
    q3_sum += it3->second;
    q5_sum += it5->second;
  }
  // All four arms move identically in the lift presets; average anyway to
  // be robust against small per-arm tracking errors.
  separation = baseSeparation(q3_sum / 4.0, q5_sum / 4.0);
  return true;
}

// ============================================================================
//  runWithWheelLift   (concurrent arm + wheel, same planned duration)
// ============================================================================

bool CompoundCommander::runWithWheelLift(
  const std::vector<ArmStep> & steps, double lift_distance)
{
  if (!rclcpp::ok() || steps.empty()) {
    return false;
  }

  double total_duration = 0.0;
  for (const auto & step : steps) {
    total_duration += step.duration;
  }
  if (total_duration <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[CompoundCommander] runWithWheelLift: non-positive total duration");
    return false;
  }

  // The wheel profile spans the same planned duration as the arm trajectory,
  // but its phase is read from sim time (node uses use_sim_time=true, so
  // node_->now() tracks the Gazebo /clock topic).  Gazebo runs slower than
  // real time, so the profile stretches automatically with the simulation
  // and both commands start together and finish together — no hardcoded
  // scale factor is needed.
  const double wheel_duration = total_duration;

  // Velocity curve derived from the travel distance (see commander.md
  // "wheel lift 运动距离与速度曲线"): the half-sine profile
  //   v(t) = v_peak * sin(pi * t / T),  t in [0, T]
  // integrates to 2 * v_peak * T / pi, so the peak that rolls exactly
  // |lift_distance| over T is v_peak = pi * |lift_distance| / (2 * T).
  // The sign carries the rolling direction: spreading the bases apart
  // (lift_distance >= 0, e.g. home -> low) keeps the negative command sign
  // observed in RViz; contracting (lift_distance < 0) reverses it.
  const double direction = (lift_distance >= 0.0) ? -1.0 : 1.0;
  const double peak_linear_speed =
    M_PI * std::abs(lift_distance) / (2.0 * wheel_duration);
  const double peak_omega = peak_linear_speed / WheelCommander::WHEEL_RADIUS;

  std::thread wheel_thread;
  if (peak_omega > 1e-9) {
    // Wait for a valid sim time before starting the profile.
    {
      const auto wait_start = std::chrono::steady_clock::now();
      while (rclcpp::ok() && node_->now().nanoseconds() <= 0) {
        rclcpp::spin_some(node_);
        if (std::chrono::steady_clock::now() - wait_start > std::chrono::seconds(10)) {
          RCLCPP_ERROR(node_->get_logger(),
                       "[CompoundCommander] No sim time received on /clock "
                       "after 10 s — cannot phase the wheel lift profile");
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    RCLCPP_INFO(node_->get_logger(),
                "[CompoundCommander] Starting compound motion: %zu arm step(s) "
                "over %.1f s (sim), wheel lift travel %.4f m -> "
                "w(t) = %.3f * sin(pi * t / %.1f) rad/s with t = sim time",
                steps.size(), total_duration, lift_distance,
                direction * peak_omega, wheel_duration);

    // ---- Wheel lift profile (background thread, sim-clock phased) --------
    // Runs concurrently with the arm steps for exactly wheel_duration of SIM
    // time, then publishes zero and exits.  The phase t comes from sim time
    // (node_->now()); the wall clock only paces the publish rate and guards
    // against a stalled sim clock.
    wheel_thread = std::thread([this, direction, peak_omega, wheel_duration]() {
      constexpr double kPeriod = 0.1;  // publish period, 10 Hz (wall clock)
      const int64_t sim_start_ns = this->node_->now().nanoseconds();
      const auto wall_start = std::chrono::steady_clock::now();
      // Generous upper bound in case sim time stalls (paused sim, no /clock).
      const double wall_limit = wheel_duration * 10.0 + 30.0;
      auto last_publish = wall_start;

      while (rclcpp::ok()) {
        const auto now = std::chrono::steady_clock::now();
        const double wall_elapsed =
          std::chrono::duration<double>(now - wall_start).count();
        if (wall_elapsed > wall_limit) {
          RCLCPP_WARN(this->node_->get_logger(),
                      "[CompoundCommander] Wheel lift aborted: sim clock stalled "
                      "(no progress within %.0f s wall time)", wall_limit);
          this->wheel_commander_.publishLiftVelocity(0.0);
          break;
        }

        const double t =
          static_cast<double>(this->node_->now().nanoseconds() - sim_start_ns) * 1e-9;

        if (t >= wheel_duration) {
          this->wheel_commander_.publishLiftVelocity(0.0);
          break;
        }

        if (std::chrono::duration<double>(now - last_publish).count() >= kPeriod) {
          // Sign: the wheel rotation direction observed in RViz is opposite
          // to the joint's positive direction when the bases spread apart.
          this->wheel_commander_.publishLiftVelocity(
            direction * peak_omega * std::sin(M_PI * t / wheel_duration));
          last_publish = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    });
  } else {
    // No wheel travel required by this preset (e.g. home / rhombus_*):
    // keep the wheels still instead of running a profile.
    RCLCPP_INFO(node_->get_logger(),
                "[CompoundCommander] Starting compound motion: %zu arm step(s) "
                "over %.1f s (sim), no wheel lift travel needed — wheels stay still",
                steps.size(), total_duration);
    wheel_commander_.publishLiftVelocity(0.0);
  }

  // ---- Run arm steps (main thread, sim-time based) --------------------
  bool arm_ok = true;
  for (size_t i = 0; i < steps.size(); ++i) {
    RCLCPP_INFO(node_->get_logger(),
                "[CompoundCommander] Arm step %zu/%zu (%.1f s) ...",
                i + 1, steps.size(), steps[i].duration);
    if (!arm_commander_.sendPositionGoal(steps[i].targets, steps[i].duration)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[CompoundCommander] Arm step %zu failed", i + 1);
      arm_ok = false;
      break;
    }
    RCLCPP_INFO(node_->get_logger(),
                "[CompoundCommander] Arm step %zu/%zu succeeded",
                i + 1, steps.size());
  }

  // ---- Wait for wheel thread to finish ---------------------------------
  if (wheel_thread.joinable()) {
    wheel_thread.join();
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CompoundCommander] Wheel lift profile finished");

  return arm_ok;
}

}  // namespace robot_commander
