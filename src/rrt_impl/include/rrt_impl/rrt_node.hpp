#ifndef RRT_NODE_HPP
#define RRT_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rrt_impl/rrt.hpp"
#include "rrt_impl/rrt_star.hpp"

#include <vector>
#include <string>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "custom_msgs/msg/goal_command.hpp"
#include <chrono>

#include "ruckig.hpp"

using GoalCommand = custom_msgs::msg::GoalCommand;

class RRTNode : public rclcpp::Node
{
public:
    RRTNode();

private:
    std::string robot_name_;
    int dof_;
    void run(uint8_t planner_type);

    rclcpp::Subscription<GoalCommand>::SharedPtr goal_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tcp_pub_;

    void goalCallback(const GoalCommand::SharedPtr msg);
    void publishTCPPath(const std::vector<Eigen::Vector3d> &tcp_path);

    // --- RRT Core ---
    std::shared_ptr<rrt_core::RRT> rrt_;
    std::shared_ptr<rrt_core::RRTStar> rrt_star_;

    std::shared_ptr<collision::CheckCollision> collision_checker_;

    // --- Configurations ---
    std::vector<double> q_start_;
    std::vector<double> q_goal_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;

    // Kinematic limits — tune these for your robot   //for panda arm
    std::vector<double> max_vel = {2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610}; // rad/s
    std::vector<double> max_acc = {15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};         // rad/s²
    std::vector<double> max_jerk = {7500, 3750, 5000, 6250, 7500, 10000, 10000};     // rad/s³

    std::shared_ptr<rrt_core::RuckigSmoother> smoother_;

    std::vector<Eigen::Vector3d> tcp_path;

    int max_iterations_;

    // --- Path result ---
    std::vector<std::vector<double>> path_;
};

#endif // RRT_NODE_HPP