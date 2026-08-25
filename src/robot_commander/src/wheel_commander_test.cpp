#include <robot_commander/wheel_commander.hpp>

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ---- Parse command-line arguments --------------------------------
  std::string mode = "forward";
  double linear_speed = 0.1;   // m/s
  double duration = 1.0;       // s
  double turn_angle = 90.0;    // deg (turn_angle mode)

  if (argc > 1) { mode = argv[1]; }
  if (mode == "turn_angle") {
    if (argc > 2) { turn_angle = std::stod(argv[2]); }
    if (argc > 3) { linear_speed = std::stod(argv[3]); }
  } else {
    if (argc > 2) { linear_speed = std::stod(argv[2]); }
    if (argc > 3) { duration = std::stod(argv[3]); }
  }

  if (mode != "forward" && mode != "turn_right_speed" && mode != "turn_left_speed" &&
      mode != "turn_angle")
  {
    fprintf(stderr,
            "Usage:  ros2 run robot_commander wheel_commander_test <mode> [args...]\n"
            "  mode      'forward', 'turn_right_speed', 'turn_left_speed' or\n"
            "            'turn_angle' (required)\n"
            "\n"
            "  forward / turn_right_speed / turn_left_speed:\n"
            "    speed     linear speed in m/s (default 0.1)\n"
            "    duration  movement duration in seconds (default 1.0)\n"
            "\n"
            "  turn_angle:\n"
            "    angle     turn angle in degrees, positive = counter-clockwise\n"
            "              (left), negative = clockwise (right) (default 90)\n"
            "    speed     linear speed in m/s (default 0.1); the duration is\n"
            "              derived from the angle and the turn radius\n"
            "\n"
            "  forward          wheel_joint_1,2 rotate forward,\n"
            "                   wheel_joint_3,4 rotate in reverse\n"
            "  turn_right_speed all four wheels rotate at the same speed (right turn)\n"
            "  turn_left_speed  all four wheels rotate at the same speed in reverse\n"
            "  turn_angle       turns in place by the given angle; requires\n"
            "                   veer position=turn, veer/arm status=free,\n"
            "                   arm position=home and wheel status=free\n");
    return 1;
  }

  // ---- Create node ------------------------------------------------
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("use_sim_time", true);
  auto node = std::make_shared<rclcpp::Node>("wheel_command", node_options);

  // ---- Create commander and drive ----------------------------------
  robot_commander::WheelCommander commander(node);

  if (!commander.waitForJointStates()) { return 1; }

  bool ok = false;
  if (mode == "forward") {
    ok = commander.driveForward(linear_speed, duration);
  } else if (mode == "turn_right_speed") {
    ok = commander.turnRightWithSpeed(linear_speed, duration);
  } else if (mode == "turn_left_speed") {
    ok = commander.turnLeftWithSpeed(linear_speed, duration);
  } else if (mode == "turn_angle") {
    ok = commander.turnByAngle(turn_angle, linear_speed);
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
