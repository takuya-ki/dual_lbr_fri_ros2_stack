#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("hello_moveit", options);

  // 🔽 use_sim_time を明示的に宣言（例外が出ても安全にスキップ）
  try {
    node->declare_parameter("use_sim_time", false);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    RCLCPP_WARN(node->get_logger(), "use_sim_time already declared, skipping");
  }

  RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface for lbr_left");

  moveit::planning_interface::MoveGroupInterface left_group(
    node,
    moveit::planning_interface::MoveGroupInterface::Options(
      "lbr_left", "robot_description", "")
  );

  RCLCPP_INFO(node->get_logger(), "Waiting for current state...");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto current_state = left_group.getCurrentState(10.0);
  if (!current_state)
  {
    RCLCPP_ERROR(node->get_logger(), "Current state NOT available");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Current state received");

  left_group.setStartStateToCurrentState();
  left_group.setPlanningTime(10.0);
  left_group.setMaxVelocityScalingFactor(1.0);
  left_group.setMaxAccelerationScalingFactor(1.0);
  left_group.setPlannerId("RRTConnect");

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.6;
  left_group.setPoseTarget(target_pose, "lbr_left_link_ee");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (left_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Plan success, executing...");
    left_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Left arm planning failed");
  }

  rclcpp::shutdown();
  return 0;
}