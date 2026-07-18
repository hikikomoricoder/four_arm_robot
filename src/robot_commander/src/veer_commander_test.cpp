#include <robot_commander/veer_commander.hpp>

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

  if (mode != "home" && mode != "forward" && mode != "turn") {
    fprintf(stderr,
            "Usage:  ros2 run robot_commander veer_commander_test <mode> [duration]\n"
            "  mode      'home', 'forward' or 'turn' (required)\n"
            "  duration  movement duration in seconds (default 2.0)\n"
            "\n"
            "  home      all veer joints to URDF zero (0 rad)\n"
            "  forward   switch to forward/backward state:\n"
            "            j1 stays, j2 +pi/2, j3 stays, j4 +pi/2\n"
            "  turn      all veer joints to +45° (pi/4 rad)\n");
    return 1;
  }

  // ---- Create node ------------------------------------------------
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("use_sim_time", true);
  auto node = std::make_shared<rclcpp::Node>("veer_commander_test", node_options);

  // ---- Create commander and run command ----------------------------
  robot_commander::VeerCommander commander(node);

  if (!commander.waitForJointStates()) { return 1; }
  if (!commander.waitForActionServer()) { return 1; }

  bool ok = false;
  if (mode == "home") {
    ok = commander.goHome(duration);
  } else if (mode == "forward") {
    ok = commander.setForwardState(duration);
  } else {
    ok = commander.setTurnState(duration);
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
