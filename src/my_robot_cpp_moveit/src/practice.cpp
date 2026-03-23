#include <memory>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <future>
#include <chrono>
#include <thread>

// Core ROS2 client library
#include "rclcpp/rclcpp.hpp"

// ROS2 action client support
#include "rclcpp_action/rclcpp_action.hpp"

// Pose message for navigation goals
#include "geometry_msgs/msg/pose_stamped.hpp"

// Nav2 action definition
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav=rclcpp_action::ClientGoalHandle<NavigateToPose>;

class ButtlerManager:public rclcpp::Node{
    public:
    ButlerManager():Node("buttler_manager"){
    nav_client=rclcpp_action::create_client<NavigateToPose>(this,"navigate");
    RCLCPP_INFO(get_logger(), "Waiting for NavigateToPose action server...");
    nav_client_->wait_for_action_server();

    init_locations();

    RCLCPP_INFO(get_logger(), "Butler Task Manager initialized.");
  
    }
    
}