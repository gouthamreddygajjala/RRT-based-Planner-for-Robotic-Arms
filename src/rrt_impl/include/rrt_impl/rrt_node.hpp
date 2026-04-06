#ifndef RRT_NODE_HPP
#define RRT_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rrt_impl/rrt.hpp"

#include <vector>
#include <string>
#include "std_msgs/msg/float64_multi_array.hpp"

class RRTNode : public rclcpp::Node
{
public:
    RRTNode();

private:
    std::string robot_name_;
    int dof_;
    void run();

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_sub_;

    void goalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // --- RRT Core ---
    std::shared_ptr<rrt_core::RRT> rrt_;

    // --- Configurations ---
    std::vector<double> q_start_;
    std::vector<double> q_goal_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;

    int max_iterations_;

    // --- Path result ---
    std::vector<std::vector<double>> path_;
};

#endif // RRT_NODE_HPP