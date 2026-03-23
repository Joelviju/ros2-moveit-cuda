#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <fstream>
#include <chrono>
#include <thread>

#include "gpu_collision.hpp"

// ======================================================
// ROBOT SPHERES
// ======================================================
std::vector<Sphere> getRobotSpheres(
    moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<Sphere> spheres;

    auto state = move_group.getCurrentState();
    state->update();

    std::vector<std::string> links = {
        "robot1_shoulder_link",
        "robot1_upper_arm_link",
        "robot1_forearm_link",
        "robot1_wrist_1_link",
        "robot1_wrist_2_link",
        "robot1_wrist_3_link"
    };

    for (auto& link : links)
    {
        const auto& tf = state->getGlobalLinkTransform(link);

        Sphere s;
        s.x = tf.translation().x();
        s.y = tf.translation().y();
        s.z = tf.translation().z();
        s.radius = 0.02;

        spheres.push_back(s);
    }

    return spheres;
}

// ======================================================
// OBSTACLES
// ======================================================
std::vector<Sphere> getObstacles()
{
    std::vector<Sphere> obs;

    for (int i = 0; i < 6; ++i)
    {
        Sphere s;
        s.x = 0.4 + 0.05 * i;
        s.y = 0.0;
        s.z = 0.2;
        s.radius = 0.03;
        obs.push_back(s);
    }

    return obs;
}

// ======================================================
// MAIN
// ======================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "gpu_planning_benchmark",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread([&exec]() { exec.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

    move_group.setPlanningTime(5.0);
    move_group.setMaxVelocityScalingFactor(1.0);

    geometry_msgs::msg::Pose target;
    target.position.x = 0.4;
    target.position.y = 0.0;
    target.position.z = 0.5;
    target.orientation.w = 1.0;

    std::ofstream file("/workspaces/ros2_CUDA/gpu_benchmark.csv");
    file << "run,planning_time,execution_time,trajectory_points,success\n";

    auto obstacles = getObstacles();

    for (int i = 0; i < 50; ++i)
    {
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(target);

        auto robot_spheres = getRobotSpheres(move_group);

        auto t0 = std::chrono::high_resolution_clock::now();
        bool collision = gpuCollisionCheck(robot_spheres, obstacles);
        auto t1 = std::chrono::high_resolution_clock::now();

        double gpu_time =
            std::chrono::duration<double>(t1 - t0).count();

        if (collision)
        {
            file << i << ",0,0,0,0\n";
            continue;
        }

        auto t2 = std::chrono::high_resolution_clock::now();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success =
            (move_group.plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        auto t3 = std::chrono::high_resolution_clock::now();

        double plan_time =
            std::chrono::duration<double>(t3 - t2).count();

        double exec_time = 0;
        int pts = 0;

        if (success)
        {
            pts = plan.trajectory_.joint_trajectory.points.size();

            auto e1 = std::chrono::high_resolution_clock::now();
            move_group.execute(plan);
            auto e2 = std::chrono::high_resolution_clock::now();

            exec_time =
                std::chrono::duration<double>(e2 - e1).count();
        }

        file << i << ","
             << plan_time << ","
             << exec_time << ","
             << pts << ","
             << success << "\n";

        RCLCPP_INFO(node->get_logger(),
            "[%d] GPU %.6f | Plan %.3f | Exec %.3f",
            i, gpu_time, plan_time, exec_time);
    }

    file.close();

    rclcpp::shutdown();
    return 0;
}