#include "rrt_impl/rrt_node.hpp"
#include <sstream>

RRTNode::RRTNode() : Node("rrt_node"), max_iterations_(1000)
{
    q_start_ = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};

    goal_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/goal_command", 10,
                                                                            std::bind(&RRTNode::goalCallback, this, std::placeholders::_1));

    tcp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_path", 10);

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

    smoother_ = std::make_shared<rrt_core::RuckigSmoother>(dof_, 0.01);

    collision_checker_ = std::make_shared<collision::CheckCollision>(urdf); // only to compute path length, not used in RRT (can be used for future optimizations)

    RCLCPP_INFO(this->get_logger(), "RRT Node initialized. Waiting for goal...");
}

void RRTNode::run()
{
    rrt_->setStart(q_start_);
    rrt_->setGoal(q_goal_);

    auto start = std::chrono::high_resolution_clock::now();

    bool success = rrt_->plan(max_iterations_);

    auto end = std::chrono::high_resolution_clock::now();

    double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Path found!");

        path_ = rrt_->getPath();
        // path_ = rrt_->shortcutPath(path_);
        // path_ = rrt_->interpolatePath(path_, 200); // cubic spline, doesnt work

        RCLCPP_INFO(this->get_logger(), "Total way points: %ld", path_.size());

        // auto traj = smoother_->smoothPath(path_, max_vel, max_acc, max_jerk);
        // auto traj = smoother_->timeParametrize(path_, max_vel, max_acc, max_jerk);

        // path_ = traj.positions; // smooth, jerk-limited positions

        // RCLCPP_INFO(this->get_logger(), "Trajectory duration: %.3f s, Points: %ld", traj.duration, path_.size());

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

    double length = collision_checker_->computeCartesianPathLength(path_, tcp_path);

    publishTCPPath(tcp_path);

    RCLCPP_INFO(this->get_logger(), "Cartesian path length: %.4f meters, Planning time: %.3f ms", length, duration_ms);
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

void RRTNode::publishTCPPath(const std::vector<Eigen::Vector3d> &tcp_path)
{
    visualization_msgs::msg::Marker line;

    line.header.frame_id = "world"; // or "panda_link0"
    line.header.stamp = this->now();
    line.ns = "tcp_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;

    line.scale.x = 0.01; // line thickness

    line.color.r = 1.0;
    line.color.g = 0.0;
    line.color.b = 0.0;
    line.color.a = 1.0;

    for (const auto &p : tcp_path)
    {
        geometry_msgs::msg::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        line.points.push_back(pt);
    }

    tcp_pub_->publish(line);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RRTNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}