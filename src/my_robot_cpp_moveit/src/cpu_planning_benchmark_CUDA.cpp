/*#include <rclcpp/rclcpp.hpp>
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
} */


// cpu_planning_benchmark_CUDA.cpp

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>

#include <chrono>
#include <fstream>
#include <vector>

#include "gpu_collision.hpp"
#include "batch_manager.hpp"

using namespace std::chrono_literals;

// ============================================================
// 🔧 CONFIG
// ============================================================

static const int NUM_LINKS = 6;
static const int BATCH_SIZE = 256;

// ============================================================
// 🔵 GET ROBOT SPHERES
// ============================================================

std::vector<Sphere> getRobotSpheres(const moveit::core::RobotState& state)
{
    std::vector<Sphere> spheres;

    std::vector<std::string> links = {
        "robot1_shoulder_link",
        "robot1_upper_arm_link",
        "robot1_forearm_link",
        "robot1_wrist_1_link",
        "robot1_wrist_2_link",
        "robot1_wrist_3_link"
    };

    std::map<std::string, float> radii = {
        {"robot1_shoulder_link", 0.07f},
        {"robot1_upper_arm_link", 0.06f},
        {"robot1_forearm_link", 0.05f},
        {"robot1_wrist_1_link", 0.04f},
        {"robot1_wrist_2_link", 0.04f},
        {"robot1_wrist_3_link", 0.03f},
    };

    for (const auto& link : links)
    {
        const auto& tf = state.getGlobalLinkTransform(link);

        Sphere s;
        s.x = tf.translation().x();
        s.y = tf.translation().y();
        s.z = tf.translation().z();
        s.radius = radii[link];

        spheres.push_back(s);
    }

    return spheres;
}

// ============================================================
// 🔴 CREATE OBSTACLES
// ============================================================

std::vector<Sphere> createObstacles()
{
    std::vector<Sphere> obs;

    for (int i = 0; i < 6; i++)
    {
        Sphere s;
        s.x = 0.4 + 0.05 * i;
        s.y = 0.0;
        s.z = 0.2;
        s.radius = 0.05;

        obs.push_back(s);
    }

    return obs;
}

// ============================================================
// 🧠 MAIN
// ============================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("gpu_batch_planner");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

    // ========================================================
    // ⏳ WAIT FOR ROBOT STATE (CRITICAL FIX)
    // ========================================================

    move_group.startStateMonitor();
    rclcpp::sleep_for(std::chrono::seconds(2));

    moveit::core::RobotStatePtr current_state;

    RCLCPP_INFO(node->get_logger(), "Waiting for valid robot state...");

    while (!current_state)
    {
        current_state = move_group.getCurrentState(2.0);

        if (!current_state)
        {
            RCLCPP_WARN(node->get_logger(), "Still waiting for robot state...");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
        }
    }

    RCLCPP_INFO(node->get_logger(), "Robot state received!");

    // ========================================================
    // 🚀 INIT GPU
    // ========================================================

    initGPU();

    BatchManager batch(BATCH_SIZE, NUM_LINKS);
    std::vector<Sphere> obstacles = createObstacles();

    std::ofstream file("gpu_batch_results.csv");
    file << "iteration,planning_time,execution_time\n";

    // ========================================================
    // 🔁 LOOP
    // ========================================================

    for (int i = 0; i < 50; i++)
    {
        RCLCPP_INFO(node->get_logger(), "Iteration %d", i);

        auto current_state = move_group.getCurrentState();

        if (!current_state)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get robot state");
            continue;
        }

        current_state->update();  // 🔥 FIX: clean transforms

        moveit::core::RobotState start_state(*current_state);

        auto state_spheres = getRobotSpheres(start_state);

        // ====================================================
        // 🚀 ADD TO BATCH
        // ====================================================

        batch.addState(state_spheres);

        if (batch.isReady())
        {
            auto t0 = std::chrono::high_resolution_clock::now();

            batch.processBatch(obstacles);

            auto t1 = std::chrono::high_resolution_clock::now();
            double gpu_time = std::chrono::duration<double>(t1 - t0).count();

            const auto& results = batch.getResults();

            int valid_count = 0;

            for (int j = 0; j < results.size(); j++)
            {
                if (results[j] != 0) valid_count++;   // 🔥 FIXED
            }

            RCLCPP_INFO(node->get_logger(),
                        "Batch processed: %ld states | valid: %d | gpu_time: %.6f",
                        results.size(), valid_count, gpu_time);

            batch.clear();
        }

        // ====================================================
        // 🎯 PLAN
        // ====================================================

        move_group.setStartStateToCurrentState();
        move_group.setRandomTarget();

        auto t_start = std::chrono::high_resolution_clock::now();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        auto t_end = std::chrono::high_resolution_clock::now();
        double planning_time = std::chrono::duration<double>(t_end - t_start).count();

        double execution_time = 0.0;

        if (success)
        {
            auto exec_start = std::chrono::high_resolution_clock::now();

            move_group.execute(plan);

            auto exec_end = std::chrono::high_resolution_clock::now();
            execution_time = std::chrono::duration<double>(exec_end - exec_start).count();
        }

        file << i << "," << planning_time << "," << execution_time << "\n";
    }

    // ========================================================
    // 🧹 CLEANUP
    // ========================================================

    file.close();

    freeGPU();

    rclcpp::shutdown();
    spinner.join();

    return 0;
}