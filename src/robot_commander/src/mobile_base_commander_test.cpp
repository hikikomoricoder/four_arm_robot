#include <robot_commander/mobile_base_commander.hpp>

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // ---- Parse command-line arguments --------------------------------
  std::string mode;
  double param1 = 0.0;
  double param2 = 0.0;
  double param3 = 0.1;       // default linear speed (m/s)

  if (argc < 2) {
    fprintf(stderr,
            "Usage:  ros2 run robot_commander mobile_base_commander_test <mode> <param1> <param2> [param3]\n"
            "  mode     'direction_method_1' (required)\n"
            "\n"
            "  direction_method_1:\n"
            "    param1  forward displacement x (m)\n"
            "    param2  leftward displacement y (m)\n"
            "    param3  linear speed in m/s (default 0.1)\n"
            "\n"
            "Moves to the position (x, y) relative to the current one:\n"
            "first turns in place to heading atan2(y, x), then drives\n"
            "straight for hypot(x, y).  Requires: arm position=home,\n"
            "arm/veer/wheel status=free (checked via /group_state_manager).\n"
            "Afterwards the veer joints are restored to home.\n"
            "\n"
            "  e.g.  ros2 run robot_commander mobile_base_commander_test direction_method_1 0.5 0.3\n"
            "        ros2 run robot_commander mobile_base_commander_test direction_method_1 0.0 -0.4 0.15\n");
    return 1;
  }

  mode = argv[1];

  if (mode != "direction_method_1") {
    fprintf(stderr,
            "Unknown mode '%s'.  Supported: direction_method_1\n",
            mode.c_str());
    return 1;
  }

  if (argc < 4) {
    fprintf(stderr,
            "direction_method_1 requires param1 (x) and param2 (y).\n"
            "Usage:  ros2 run robot_commander mobile_base_commander_test direction_method_1 <x> <y> [speed]\n");
    return 1;
  }

  param1 = std::stod(argv[2]);  // x
  param2 = std::stod(argv[3]);  // y
  if (argc > 4) { param3 = std::stod(argv[4]); }  // speed

  // ---- Create node ------------------------------------------------
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("use_sim_time", true);
  auto node = std::make_shared<rclcpp::Node>("mobile_base_commander_test", node_options);

  // ---- Create commander and move -----------------------------------
  robot_commander::MobileBaseCommander commander(node);

  if (!commander.waitForJointStates()) { return 1; }
  if (!commander.waitForActionServer()) { return 1; }

  bool ok = false;
  if (mode == "direction_method_1") {
    ok = commander.moveToPosition(param1, param2, param3);
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
