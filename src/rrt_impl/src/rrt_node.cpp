#include "rrt_impl/rrt_node.hpp"
#include <sstream>

RRTNode::RRTNode() : Node("rrt_node"),
                     max_iterations_(1000)
{
    q_start_ = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};

    goal_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/goal_command", 10,
                                                                            std::bind(&RRTNode::goalCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("use_arm", "");
    std::string use_arm = this->get_parameter("use_arm").as_string();

    std::string prefix = "robots." + use_arm + ".";

    this->declare_parameter<int>(prefix + "dof", 0);
    this->declare_parameter<std::string>(prefix + "urdf", "");
    this->declare_parameter<std::vector<double>>(prefix + "joint_limits.lower", {});
    this->declare_parameter<std::vector<double>>(prefix + "joint_limits.upper", {});

    // 3. Read them
    int dof = this->get_parameter(prefix + "dof").as_int();
    std::string urdf = this->get_parameter(prefix + "urdf").as_string();
    lower_limits_ = this->get_parameter(prefix + "joint_limits.lower").as_double_array();
    upper_limits_ = this->get_parameter(prefix + "joint_limits.upper").as_double_array();

    RCLCPP_INFO(this->get_logger(), "Loaded robot: %s | DOF: %d | URDF: %s",
                use_arm.c_str(), dof, urdf.c_str());

    if (use_arm.empty())
    {
        RCLCPP_FATAL(this->get_logger(), "use_arm is empty!");
        throw std::runtime_error("Missing use_arm");
    }

    if (dof <= 0)
    {
        RCLCPP_FATAL(this->get_logger(), "Invalid DOF!");
        throw std::runtime_error("Invalid DOF");
    }

    if (lower_limits_.size() != upper_limits_.size())
    {
        RCLCPP_FATAL(this->get_logger(), "Joint limits mismatch!");
        throw std::runtime_error("Joint limits error");
    }

    dof_ = dof;

    rrt_ = std::make_shared<rrt_core::RRT>(dof, urdf);
    rrt_->setJointLimits(lower_limits_, upper_limits_);

    RCLCPP_INFO(this->get_logger(), "RRT Node initialized. Waiting for goal...");
}

void RRTNode::run()
{
    rrt_->setStart(q_start_);
    rrt_->setGoal(q_goal_);

    bool success = rrt_->plan(max_iterations_);

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Path found!");

        path_ = rrt_->getPath();

        RCLCPP_INFO(this->get_logger(), "Path length: %ld", path_.size());

        for (size_t i = 0; i < path_.size(); ++i)
        {
            std::ostringstream oss;
            oss << "Waypoint " << i << ": [ ";

            for (double val : path_[i])
            {
                oss << val << " ";
            }

            oss << "]";

            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Planning failed!");
    }
}

void RRTNode::goalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != static_cast<size_t>(dof_))
    {
        RCLCPP_WARN(this->get_logger(), "Invalid goal size! Expected %d DOF.", dof_);
        return;
    }

    q_goal_ = msg->data;

    RCLCPP_INFO(this->get_logger(), "Received new goal. Starting planning...");

    run();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RRTNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}