#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cartesian_move_cpp");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.setMaxAccelerationScalingFactor(0.3);

  // Get current pose
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Waypoint 1: Move forward
  geometry_msgs::msg::Pose target_pose1 = start_pose;
  target_pose1.position.z += 0.1;
  waypoints.push_back(target_pose1);

  // Waypoint 2: Move sideways
  geometry_msgs::msg::Pose target_pose2 = target_pose1;
  target_pose2.position.y += 0.1;
  waypoints.push_back(target_pose2);

  // Waypoint 3: Move down
  geometry_msgs::msg::Pose target_pose3 = target_pose2;
  target_pose3.position.z -= 0.1;
  waypoints.push_back(target_pose3);

  moveit_msgs::msg::RobotTrajectory trajectory;

  const double eef_step = 0.01;
  const double jump_threshold = 0.0;

  double fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory);

  RCLCPP_INFO(node->get_logger(), "Cartesian path success: %.2f%%", fraction * 100.0);

  if (fraction > 0.9)
  {
    move_group.execute(trajectory);
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Cartesian path planning failed.");
  }

  rclcpp::shutdown();
  return 0;
}
