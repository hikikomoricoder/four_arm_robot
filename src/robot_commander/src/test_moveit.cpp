#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // use_sim_time must be true when running alongside Gazebo so that the
    // current state monitor accepts joint state timestamps from sim clock.
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.append_parameter_override("use_sim_time", true);
    auto node = std::make_shared<rclcpp::Node>("test_moveit", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create a MoveGroupInterface object for the robot
    auto veer = moveit::planning_interface::MoveGroupInterface(node, "veer");
    veer.setMaxVelocityScalingFactor(0.5);
    veer.setMaxAccelerationScalingFactor(0.5);
    // Joint goal
    std::vector<double> veer_joint_values = {1.0, 1.0, -1.0, 1.0};
    veer.setStartStateToCurrentState();
    veer.setJointValueTarget(veer_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan test_plan_1;
    bool success = (veer.plan(test_plan_1) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
        veer.execute(test_plan_1);
    }
    else {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan joint goal");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}