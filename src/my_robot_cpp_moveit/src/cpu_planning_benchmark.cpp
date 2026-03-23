/*#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <fstream>
#include <thread>
#include <random>
#include <cmath>

// ================================================================
// REPRODUCIBLE CLUTTER GENERATION
// ================================================================
void addDeterministicClutter(
    moveit::planning_interface::PlanningSceneInterface& psi,
    int num_objects)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    std::mt19937 gen(42);  // fixed seed for reproducibility

    std::uniform_real_distribution<> x_dist(0.30, 0.65);
    std::uniform_real_distribution<> y_dist(-0.4, 0.4);
    std::uniform_real_distribution<> z_dist(0.05, 0.55);
    std::uniform_real_distribution<> size_dist(0.04, 0.09);

    for (int i = 0; i < num_objects; ++i)
    {
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = "world";
        object.id = "box_" + std::to_string(i);

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {size_dist(gen),
                                size_dist(gen),
                                size_dist(gen)};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = x_dist(gen);
        pose.position.y = y_dist(gen);
        pose.position.z = z_dist(gen);

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;

        collision_objects.push_back(object);
    }

    psi.applyCollisionObjects(collision_objects);
}

// ================================================================
// JOINT-SPACE EUCLIDEAN PATH LENGTH
// ================================================================
double computeJointPathLength(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    const auto& traj = plan.trajectory_.joint_trajectory.points;

    if (traj.size() < 2)
        return 0.0;

    double length = 0.0;

    for (size_t i = 1; i < traj.size(); ++i)
    {
        const auto& curr = traj[i].positions;
        const auto& prev = traj[i - 1].positions;

        double segment = 0.0;

        for (size_t j = 0; j < curr.size(); ++j)
        {
            double diff = curr[j] - prev[j];
            segment += diff * diff;
        }

        length += std::sqrt(segment);
    }

    return length;
}

// ================================================================
// CARTESIAN END-EFFECTOR PATH LENGTH
// ================================================================
  double computeCartesianPathLength(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    const auto* jmg =
        move_group.getCurrentState()->getJointModelGroup(
            move_group.getName());

    moveit::core::RobotState state(*move_group.getCurrentState());

    double length = 0.0;
    bool first = true;
    Eigen::Vector3d prev_position;

    for (const auto& point :
         plan.trajectory_.joint_trajectory.points)
    {
        state.setJointGroupPositions(jmg, point.positions);
        state.update();

        const Eigen::Isometry3d& ee =
            state.getGlobalLinkTransform(
                move_group.getEndEffectorLink());

        Eigen::Vector3d position = ee.translation();

        if (!first)
            length += (position - prev_position).norm();

        prev_position = position;
        first = false;
    }

    return length;
}
// ================================================================
// MAIN BENCHMARK
// ================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "cpu_planning_benchmark",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(5.0);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    RCLCPP_INFO(node->get_logger(), "Planning frame: %s",
                move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(node->get_logger(), "End effector: %s",
                move_group.getEndEffectorLink().c_str());

    rclcpp::sleep_for(std::chrono::seconds(2));

    // 🔥 Add deterministic clutter
    addDeterministicClutter(planning_scene_interface, 15);
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 🔥 Safe reachable target
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.35;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.6;

    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.7071;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.7071;

    std::ofstream file("/workspaces/ros2_CUDA/cpu_benchmark.csv");
    file << "run,planning_time,execution_time,trajectory_points,"
            "joint_path_length,cartesian_path_length,success\n";

    const int runs = 50;

    for (int i = 0; i < runs; ++i)
    {
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(target_pose);

        auto start_plan = std::chrono::high_resolution_clock::now();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success =
            (move_group.plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        auto end_plan = std::chrono::high_resolution_clock::now();

        double planning_time =
            std::chrono::duration<double>(
                end_plan - start_plan).count();

        double execution_time = 0.0;
        int trajectory_points = 0;
        double joint_length = 0.0;
        double cartesian_length = 0.0;

        if (success)
        {
            trajectory_points =
                plan.trajectory_.joint_trajectory.points.size();

            joint_length =
                computeJointPathLength(plan);

            cartesian_length =
                computeCartesianPathLength(move_group, plan);

            auto start_exec =
                std::chrono::high_resolution_clock::now();

            move_group.execute(plan);

            auto end_exec =
                std::chrono::high_resolution_clock::now();

            execution_time =
                std::chrono::duration<double>(
                    end_exec - start_exec).count();
        }

        file << i << ","
             << planning_time << ","
             << execution_time << ","
             << trajectory_points << ","
             << joint_length << ","
             << cartesian_length << ","
             << success << "\n";

        RCLCPP_INFO(node->get_logger(),
                    "Run %d | Plan: %.3f s | Exec: %.3f s | "
                    "Pts: %d | JLen: %.3f | CLen: %.3f | %s",
                    i,
                    planning_time,
                    execution_time,
                    trajectory_points,
                    joint_length,
                    cartesian_length,
                    success ? "SUCCESS" : "FAIL");

        move_group.clearPoseTargets();
    }

    file.close();

    RCLCPP_INFO(node->get_logger(),
                "Benchmark complete.");

    rclcpp::shutdown();
    return 0;
} */

/*
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <fstream>
#include <thread>
#include <random>
#include <cmath>

// ================================================================
// CONTROLLED CLUTTER (FIXED VERSION)
// ================================================================
void addDeterministicClutter(
    moveit::planning_interface::PlanningSceneInterface& psi,
    int num_objects)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    std::mt19937 gen(42);

    std::uniform_real_distribution<> x_dist(0.35, 0.65);
    std::uniform_real_distribution<> y_dist(-0.3, 0.3);
    std::uniform_real_distribution<> z_dist(0.05, 0.30); // 🔥 LOWERED
    std::uniform_real_distribution<> size_dist(0.04, 0.08);

    for (int i = 0; i < num_objects; ++i)
    {
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = x_dist(gen);
        pose.position.y = y_dist(gen);
        pose.position.z = z_dist(gen);

        // 🔥 KEEP FREE SPACE ABOVE (CRITICAL)
        if (pose.position.z > 0.35)
            continue;

        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = "world";
        object.id = "box_" + std::to_string(i);

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {
            size_dist(gen),
            size_dist(gen),
            size_dist(gen)
        };

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;

        collision_objects.push_back(object);
    }

    psi.applyCollisionObjects(collision_objects);
}

// ================================================================
// JOINT PATH LENGTH
// ================================================================
double computeJointPathLength(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    const auto& traj = plan.trajectory_.joint_trajectory.points;

    if (traj.size() < 2) return 0.0;

    double length = 0.0;

    for (size_t i = 1; i < traj.size(); ++i)
    {
        double segment = 0.0;
        for (size_t j = 0; j < traj[i].positions.size(); ++j)
        {
            double diff = traj[i].positions[j] - traj[i - 1].positions[j];
            segment += diff * diff;
        }
        length += std::sqrt(segment);
    }

    return length;
}

// ================================================================
// CARTESIAN PATH LENGTH
// ================================================================
double computeCartesianPathLength(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    const auto* jmg =
        move_group.getCurrentState()->getJointModelGroup(
            move_group.getName());

    moveit::core::RobotState state(*move_group.getCurrentState());

    double length = 0.0;
    bool first = true;
    Eigen::Vector3d prev_position;

    for (const auto& point : plan.trajectory_.joint_trajectory.points)
    {
        state.setJointGroupPositions(jmg, point.positions);
        state.update();

        const auto& ee =
            state.getGlobalLinkTransform(move_group.getEndEffectorLink());

        Eigen::Vector3d pos = ee.translation();

        if (!first)
            length += (pos - prev_position).norm();

        prev_position = pos;
        first = false;
    }

    return length;
}

// ================================================================
// MAIN
// ================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "cpu_planning_benchmark",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    const std::string PLANNING_GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface psi;

    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(3); // 🔥 IMPORTANT
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    rclcpp::sleep_for(std::chrono::seconds(2));

    // 🔥 REDUCED CLUTTER
    addDeterministicClutter(psi, 6);
    rclcpp::sleep_for(std::chrono::seconds(2));

    // 🔥 SAFE TARGET
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.5;

    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.7071;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.7071;

    std::ofstream file("/workspaces/ros2_CUDA/cpu_benchmark.csv");
    file << "run,planning_time,execution_time,trajectory_points,"
            "joint_path_length,cartesian_path_length,success\n";

    const int runs = 50;

    for (int i = 0; i < runs; ++i)
    {
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(target_pose);

        // 🔥 IK CHECK (CRITICAL)
        auto state = move_group.getCurrentState(10);
        const auto* jmg = state->getJointModelGroup(PLANNING_GROUP);

        bool ik_ok = state->setFromIK(jmg, target_pose);

        if (!ik_ok)
        {
            RCLCPP_WARN(node->get_logger(), "IK FAILED — skipping run");

            file << i << ",0,0,0,0,0,0\n";
            continue;
        }

        auto start_plan = std::chrono::high_resolution_clock::now();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success =
            (move_group.plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        auto end_plan = std::chrono::high_resolution_clock::now();

        double planning_time =
            std::chrono::duration<double>(end_plan - start_plan).count();

        double execution_time = 0.0;
        int trajectory_points = 0;
        double joint_length = 0.0;
        double cartesian_length = 0.0;

        if (success)
        {
            trajectory_points =
                plan.trajectory_.joint_trajectory.points.size();

            joint_length = computeJointPathLength(plan);
            cartesian_length = computeCartesianPathLength(move_group, plan);

            auto start_exec = std::chrono::high_resolution_clock::now();
            move_group.execute(plan);
            auto end_exec = std::chrono::high_resolution_clock::now();

            execution_time =
                std::chrono::duration<double>(end_exec - start_exec).count();
        }

        file << i << ","
             << planning_time << ","
             << execution_time << ","
             << trajectory_points << ","
             << joint_length << ","
             << cartesian_length << ","
             << success << "\n";

        RCLCPP_INFO(node->get_logger(),
                    "Run %d | Plan: %.3f | Exec: %.3f | Pts: %d | %s",
                    i,
                    planning_time,
                    execution_time,
                    trajectory_points,
                    success ? "SUCCESS" : "FAIL");

        move_group.clearPoseTargets();
    }

    file.close();

    RCLCPP_INFO(node->get_logger(), "Benchmark complete.");
    rclcpp::shutdown();
    return 0;
}
*/
#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <fstream>
#include <thread>
#include <random>
#include <cmath>

// ================================================================
// CONTROLLED CLUTTER
// ================================================================
void addDeterministicClutter(
    moveit::planning_interface::PlanningSceneInterface& psi,
    int num_objects)
{
    std::vector<moveit_msgs::msg::CollisionObject> objs;

    std::mt19937 gen(42);

    std::uniform_real_distribution<> x(0.35, 0.65);
    std::uniform_real_distribution<> y(-0.3, 0.3);
    std::uniform_real_distribution<> z(0.05, 0.30);
    std::uniform_real_distribution<> s(0.04, 0.08);

    for (int i = 0; i < num_objects; ++i)
    {
        moveit_msgs::msg::CollisionObject obj;
        obj.header.frame_id = "world";
        obj.id = "box_" + std::to_string(i);

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {s(gen), s(gen), s(gen)};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = x(gen);
        pose.position.y = y(gen);
        pose.position.z = z(gen);

        obj.primitives.push_back(prim);
        obj.primitive_poses.push_back(pose);
        obj.operation = obj.ADD;

        objs.push_back(obj);
    }

    psi.applyCollisionObjects(objs);
}

// ================================================================
// SAFE STATE FETCH
// ================================================================
moveit::core::RobotStatePtr getSafeState(
    moveit::planning_interface::MoveGroupInterface& move_group,
    rclcpp::Logger logger)
{
    for (int i = 0; i < 3; ++i)
    {
        auto state = move_group.getCurrentState(1.0);
        if (state) return state;

        RCLCPP_WARN(logger, "Retrying state fetch...");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return nullptr;
}

// ================================================================
// WAIT FOR INITIAL STATE (HUMBLE FIX)
// ================================================================
bool waitForInitialState(
    moveit::planning_interface::MoveGroupInterface& move_group,
    rclcpp::Logger logger)
{
    RCLCPP_INFO(logger, "Waiting for joint states...");

    for (int i = 0; i < 10; ++i)
    {
        auto state = move_group.getCurrentState(0.5);

        if (state)
        {
            RCLCPP_INFO(logger, "Joint state received.");
            return true;
        }

        RCLCPP_WARN(logger, "Still waiting for joint states...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    return false;
}

// ================================================================
// MAIN
// ================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "cpu_planning_benchmark",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    // 🔥 CRITICAL FOR GAZEBO
    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    std::thread([&exec]() { exec.spin(); }).detach();

    const std::string GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(node, GROUP);
    moveit::planning_interface::PlanningSceneInterface psi;

    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(3);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    // 🔥 WAIT FOR STATE (FIXED)
    if (!waitForInitialState(move_group, node->get_logger()))
    {
        RCLCPP_ERROR(node->get_logger(), "No joint states available.");
        return 1;
    }

    rclcpp::sleep_for(std::chrono::seconds(2));

    addDeterministicClutter(psi, 6);
    rclcpp::sleep_for(std::chrono::seconds(2));

    // TARGET
    geometry_msgs::msg::Pose target;
    target.position.x = 0.4;
    target.position.y = 0.0;
    target.position.z = 0.5;

    target.orientation.x = 0.0;
    target.orientation.y = 0.7071;
    target.orientation.z = 0.0;
    target.orientation.w = 0.7071;

    std::ofstream file("/workspaces/ros2_CUDA/cpu_benchmark.csv");
    file << "run,planning_time,execution_time,trajectory_points,success\n";

    const int runs = 50;

    for (int i = 0; i < runs; ++i)
    {
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(target);

        auto state = getSafeState(move_group, node->get_logger());

        if (!state)
        {
            RCLCPP_ERROR(node->get_logger(), "Skipping run (no state)");
            file << i << ",0,0,0,0\n";
            continue;
        }

        const auto* jmg = state->getJointModelGroup(GROUP);

        if (!state->setFromIK(jmg, target))
        {
            RCLCPP_WARN(node->get_logger(), "IK failed");
            file << i << ",0,0,0,0\n";
            continue;
        }

        auto t1 = std::chrono::high_resolution_clock::now();

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success =
            (move_group.plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        auto t2 = std::chrono::high_resolution_clock::now();

        double plan_time =
            std::chrono::duration<double>(t2 - t1).count();

        double exec_time = 0.0;
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
                    "Run %d | Plan %.3f | Exec %.3f | %s",
                    i,
                    plan_time,
                    exec_time,
                    success ? "SUCCESS" : "FAIL");

        move_group.clearPoseTargets();
    }

    file.close();

    RCLCPP_INFO(node->get_logger(), "Benchmark complete.");
    rclcpp::shutdown();
    return 0;
}