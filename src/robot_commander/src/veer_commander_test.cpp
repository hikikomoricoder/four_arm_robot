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

  if (mode != "home" && mode != "forward" && mode != "turn" && mode != "lift") {
    fprintf(stderr,
            "Usage:  ros2 run robot_commander veer_commander_test <mode> [duration]\n"
            "  mode      'home', 'forward', 'turn' or 'lift' (required)\n"
            "  duration  movement duration in seconds (default 3.0)\n"
            "\n"
            "  home      all veer joints to URDF zero (0 rad)\n"
            "  forward   switch to forward/backward state:\n"
            "            first setHomeState, then j2 -pi/2, j4 -pi/2\n"
            "  turn      all veer joints to +45\u00B0 (pi/4 rad)\n"
            "  lift      all veer joints to -45\u00B0 (-pi/4 rad)\n");
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
    ok = commander.setHomeState(duration);
  } else if (mode == "forward") {
    ok = commander.setForwardState(duration);
  } else if (mode == "turn") {
    ok = commander.setTurnState(duration);
  } else {
    ok = commander.setLiftState(duration);
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
