#include <robot_commander/arm_commander.hpp>

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ---- Parse command-line arguments --------------------------------
  double duration = 3.0;       // s

  if (argc > 1) { duration = std::stod(argv[1]); }

  if (argc > 2) {
    fprintf(stderr,
            "Usage:  ros2 run robot_commander arm_commander_test [duration]\n"
            "  duration   movement duration in seconds (default 3.0)\n"
            "\n"
            "Low-level smoke test: send all 16 arm joints to the home\n"
            "position (0 rad) via ArmCommander::sendPositionGoal.\n"
            "The named presets (home/low/high/rhombus_1/rhombus_2) now live\n"
            "in CompoundCommander — use compound_commander_test for them.\n");
    return 1;
  }

  // ---- Create node ------------------------------------------------
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.append_parameter_override("use_sim_time", true);
  auto node = std::make_shared<rclcpp::Node>("arm_commander_test", node_options);

  // ---- Create commander and run command ----------------------------
  robot_commander::ArmCommander commander(node);

  if (!commander.waitForJointStates()) { return 1; }
  if (!commander.waitForActionServer()) { return 1; }

  const bool ok =
    commander.sendPositionGoal(robot_commander::ArmCommander::homePositions(), duration);

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
