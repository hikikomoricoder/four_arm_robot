#include <robot_commander/compound_commander.hpp>

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ---- Parse command-line arguments --------------------------------
  std::string mode = "home";
  double duration = 3.0;       // s

  if (argc > 1) { mode = argv[1]; }
  if (argc > 2) { duration = std::stod(argv[2]); }

  if (mode != "home" && mode != "low" && mode != "high" &&
      mode != "rhombus_1" && mode != "rhombus_2")
  {
    fprintf(stderr,
            "Usage:  ros2 run robot_commander compound_commander_test <mode> [duration]\n"
            "  mode       'home', 'low', 'high', 'rhombus_1' or 'rhombus_2' (required)\n"
            "  duration   movement duration in seconds (default 3.0)\n"
            "\n"
            "Every preset requires: arm status free, veer position lift,\n"
            "veer status free, wheel status free (checked via\n"
            "/group_state_manager).  During execution the wheels run the\n"
            "lift-mode half-sine velocity profile for the same total\n"
            "duration as the arm motion.\n"
            "\n"
            "  home       all 16 arm joints back to URDF zero (0 rad)\n"
            "  low        all arms: j3 -pi/4, j5 +pi/2, j7 -pi/4\n"
            "  high       all arms: j3 +pi/8, j5 -pi/4, j7 +pi/8\n"
            "  rhombus_1  j1 = +pi/4, -pi/4, +pi/4, -pi/4 (diamond)\n"
            "  rhombus_2  j1 = -pi/4, +pi/4, -pi/4, +pi/4 (diamond)\n");
    return 1;
  }

  // ---- Create node ------------------------------------------------
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("use_sim_time", true);
  auto node = std::make_shared<rclcpp::Node>("compound_commander_test", node_options);

  // ---- Create commander and run command ----------------------------
  robot_commander::CompoundCommander commander(node);

  if (!commander.waitForJointStates()) { return 1; }
  if (!commander.waitForActionServer()) { return 1; }

  bool ok = false;
  if (mode == "home") {
    ok = commander.setHomeState(duration);
  } else if (mode == "low") {
    ok = commander.setLowState(duration);
  } else if (mode == "high") {
    ok = commander.setHighState(duration);
  } else if (mode == "rhombus_1") {
    ok = commander.setRhombus1State(duration);
  } else {
    ok = commander.setRhombus2State(duration);
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
