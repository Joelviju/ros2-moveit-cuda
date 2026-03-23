/*#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = rclcpp::Node::make_shared("move_arm_cpp");

  // Create executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create MoveGroup Interface
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(node->get_logger(), "End effector link: %s",
              move_group.getEndEffectorLink().c_str());

  // Set target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.5;

  move_group.setPoseTarget(target_pose);
  

  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Plan successful! Executing...");
    move_group.execute(my_plan);
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
*/

/*
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <fstream>
#include <thread>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // -------- Create Node --------
  auto node = rclcpp::Node::make_shared(
      "move_to_pose_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Executor needed for MoveIt async interfaces
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // -------- MoveIt Interface --------
  const std::string PLANNING_GROUP = "manipulator";  // UR5 default group

  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  node->declare_parameter("ompl.verbose", true);

  move_group.setPlanningTime(5.0);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
 
  RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  // -------- Target Pose --------
  geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.4;
target_pose.position.y = 0.0;
target_pose.position.z = 0.6;

target_pose.orientation.x = 0.0;
target_pose.orientation.y = 1.0;
target_pose.orientation.z = 0.0;
target_pose.orientation.w = 0.0;

  move_group.setPoseTarget(target_pose);

  // -------- Planning --------
  auto start_plan = std::chrono::high_resolution_clock::now();
 rclcpp::sleep_for(std::chrono::seconds(3));
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
      (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  auto end_plan = std::chrono::high_resolution_clock::now();

  double planning_time =
      std::chrono::duration<double>(end_plan - start_plan).count();

  RCLCPP_INFO(node->get_logger(),
              "Planning %s (%.6f sec)",
              success ? "SUCCESS" : "FAILED",
              planning_time);

  // -------- Save planning time --------
  std::ofstream file("planning_results.csv", std::ios::app);

  if (file.is_open())
  {
    file << planning_time << "\n";
    file.close();
  }

  if (!success)
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed. Exiting.");
    rclcpp::shutdown();
    return 1;
  }

  // -------- Execution --------
  auto start_exec = std::chrono::high_resolution_clock::now();

  move_group.execute(plan);

  auto end_exec = std::chrono::high_resolution_clock::now();

  double execution_time =
      std::chrono::duration<double>(end_exec - start_exec).count();

  RCLCPP_INFO(node->get_logger(),
              "Execution completed in %.6f sec",
              execution_time);

  rclcpp::shutdown();
  return 0;
}*/
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <fstream>
#include <thread>
#include <numeric>
#include <vector>
#include <cmath>

double computePathLength(
    const trajectory_msgs::msg::JointTrajectory& traj)
{
  double length = 0.0;

  for (size_t i = 1; i < traj.points.size(); ++i)
  {
    double dist = 0.0;
    for (size_t j = 0; j < traj.points[i].positions.size(); ++j)
    {
      double diff =
          traj.points[i].positions[j] - traj.points[i - 1].positions[j];
      dist += diff * diff;
    }
    length += std::sqrt(dist);
  }

  return length;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
      "cpu_planning_benchmark",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(1);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
              move_group.getPlanningFrame().c_str());

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.6;
  target_pose.orientation.w = 1.0;

  const int NUM_RUNS = 50;

  std::ofstream file("cpu_benchmark.csv");
  file << "run,planning_time,execution_time,trajectory_points,path_length,success\n";

  std::vector<double> planning_times;
  std::vector<double> execution_times;

  int success_count = 0;

  for (int i = 0; i < NUM_RUNS; ++i)
  {
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto start_plan = std::chrono::high_resolution_clock::now();
    bool success =
        (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    auto end_plan = std::chrono::high_resolution_clock::now();

    double planning_time =
        std::chrono::duration<double>(end_plan - start_plan).count();

    double execution_time = 0.0;
    size_t trajectory_points = 0;
    double path_length = 0.0;

    if (success)
    {
      success_count++;

      trajectory_points =
          plan.trajectory_.joint_trajectory.points.size();

      path_length =
          computePathLength(plan.trajectory_.joint_trajectory);

      auto start_exec = std::chrono::high_resolution_clock::now();
      move_group.execute(plan);
      auto end_exec = std::chrono::high_resolution_clock::now();

      execution_time =
          std::chrono::duration<double>(end_exec - start_exec).count();

      planning_times.push_back(planning_time);
      execution_times.push_back(execution_time);
    }

    file << i << ","
         << planning_time << ","
         << execution_time << ","
         << trajectory_points << ","
         << path_length << ","
         << success << "\n";

    RCLCPP_INFO(node->get_logger(),
                "Run %d | Plan: %.4f s | Exec: %.4f s | Points: %zu | Length: %.4f | %s",
                i,
                planning_time,
                execution_time,
                trajectory_points,
                path_length,
                success ? "SUCCESS" : "FAIL");
  }

  file.close();

  double mean_plan =
      std::accumulate(planning_times.begin(), planning_times.end(), 0.0)
      / planning_times.size();

  double mean_exec =
      std::accumulate(execution_times.begin(), execution_times.end(), 0.0)
      / execution_times.size();

  RCLCPP_INFO(node->get_logger(),
              "==========================================");
  RCLCPP_INFO(node->get_logger(),
              "Success rate: %d / %d", success_count, NUM_RUNS);
  RCLCPP_INFO(node->get_logger(),
              "Mean planning time: %.6f sec", mean_plan);
  RCLCPP_INFO(node->get_logger(),
              "Mean execution time: %.6f sec", mean_exec);
  RCLCPP_INFO(node->get_logger(),
              "==========================================");

  rclcpp::shutdown();
  return 0;
}