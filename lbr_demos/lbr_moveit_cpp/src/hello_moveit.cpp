#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hello_moveit");

  moveit::planning_interface::MoveGroupInterface right_group(
      node,
      moveit::planning_interface::MoveGroupInterface::Options(
          "lbr_right",                 // planning group
          "robot_description",         // shared description
          "lbr_right"                  // namespace
      ));

  geometry_msgs::msg::Pose right_pose;
  right_pose.orientation.w = 1.0;
  right_pose.position.x = 0.4;
  right_pose.position.y = -0.3;
  right_pose.position.z = 0.9;
  right_group.setPoseTarget(right_pose);

  moveit::planning_interface::MoveGroupInterface::Plan right_plan;
  if (right_group.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    right_group.execute(right_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Right arm planning failed");
  }

  moveit::planning_interface::MoveGroupInterface left_group(
      node,
      moveit::planning_interface::MoveGroupInterface::Options(
          "lbr_left",                  // planning group
          "robot_description",         // shared description
          "lbr_left"                   // namespace
      ));

  geometry_msgs::msg::Pose left_pose;
  left_pose.orientation.w = 1.0;
  left_pose.position.x = -0.4;
  left_pose.position.y = 0.3;
  left_pose.position.z = 0.9;
  left_group.setPoseTarget(left_pose);

  moveit::planning_interface::MoveGroupInterface::Plan left_plan;
  if (left_group.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    left_group.execute(left_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Left arm planning failed");
  }

  rclcpp::shutdown();
  return 0;
}