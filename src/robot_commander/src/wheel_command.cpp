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

  if (argc > 1) { mode = argv[1]; }
  if (argc > 2) { linear_speed = std::stod(argv[2]); }
  if (argc > 3) { duration = std::stod(argv[3]); }

  if (mode != "forward" && mode != "turn") {
    fprintf(stderr,
            "Usage:  ros2 run robot_commander wheel_command <mode> [speed] [duration]\n"
            "  mode     'forward' or 'turn' (required)\n"
            "  speed     linear speed in m/s (default 0.1)\n"
            "  duration  movement duration in seconds (default 1.0)\n"
            "\n"
            "  forward  all four wheels rotate at the same speed\n"
            "  turn     wheel_joint_1,2 rotate forward,\n"
            "           wheel_joint_3,4 rotate in reverse\n");
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
  if (!commander.waitForActionServer()) { return 1; }

  bool ok = false;
  if (mode == "forward") {
    ok = commander.driveForward(linear_speed, duration);
  } else {
    ok = commander.driveTurn(linear_speed, duration);
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
