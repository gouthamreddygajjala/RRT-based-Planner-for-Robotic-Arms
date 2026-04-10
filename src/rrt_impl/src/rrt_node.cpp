#include "rrt_impl/rrt_node.hpp"
#include <sstream>

RRTNode::RRTNode() : Node("rrt_node")
{

    goal_sub_ = this->create_subscription<GoalCommand>("/goal_command", 10, std::bind(&RRTNode::goalCallback, this, std::placeholders::_1));

    tcp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_path", 10);
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

    this->declare_parameter<std::string>("use_arm", "");
    std::string use_arm = this->get_parameter("use_arm").as_string();

    std::string prefix = "robots." + use_arm + ".";

    this->declare_parameter<int>(prefix + "dof", 0);
    this->declare_parameter<std::string>(prefix + "urdf", "");
    this->declare_parameter<std::vector<double>>(prefix + "joint_limits.lower", {});
    this->declare_parameter<std::vector<double>>(prefix + "joint_limits.upper", {});
    this->declare_parameter<std::vector<double>>(prefix + "home_config", {});

    this->declare_parameter<bool>("debug", false);

    this->declare_parameter<int>("max_iterations", 1000);
    this->declare_parameter<double>("step_size", 0.06);
    this->declare_parameter<double>("goal_bias", 0.1);
    this->declare_parameter<double>("goal_tolerance", 0.1);

    int dof = this->get_parameter(prefix + "dof").as_int();
    urdf_ = this->get_parameter(prefix + "urdf").as_string();
    lower_limits_ = this->get_parameter(prefix + "joint_limits.lower").as_double_array();
    upper_limits_ = this->get_parameter(prefix + "joint_limits.upper").as_double_array();
    q_start_ = this->get_parameter(prefix + "home_config").as_double_array();

    is_debug = this->get_parameter("debug").as_bool();

    max_iterations_ = this->get_parameter("max_iterations").as_int();
    step_size = this->get_parameter("step_size").as_double();
    goal_bias = this->get_parameter("goal_bias").as_double();
    goal_tolerance = this->get_parameter("goal_tolerance").as_double();

    this->declare_parameter<std::vector<std::string>>(prefix + "joint_names", {});
    joint_names_ = this->get_parameter(prefix + "joint_names").as_string_array();

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg)
                                                                               {
        q_start_.clear();
        for (const auto &name : joint_names_)
        {
            auto it = std::find(msg->name.begin(), msg->name.end(), name);
            if (it != msg->name.end())
            {
                size_t index = std::distance(msg->name.begin(), it);
                q_start_.push_back(msg->position[index]);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Joint %s not found in joint states!", name.c_str());
                q_start_.push_back(0.0); // default to 0 if not found
            }
        } });

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

    // rrt_ = std::make_shared<rrt_core::RRT>(dof, urdf);

    // rrt_->setJointLimits(lower_limits_, upper_limits_);
    // rrt_->setStepSize(step_size);
    // rrt_->setGoalBias(goal_bias);
    // rrt_->setGoalTolerance(goal_tolerance);

    // rrt_star_ = std::make_shared<rrt_core::RRTStar>(dof_, urdf);

    // rrt_star_->setJointLimits(lower_limits_, upper_limits_);
    // rrt_star_->setStepSize(step_size);
    // rrt_star_->setGoalBias(goal_bias);
    // rrt_star_->setGoalTolerance(goal_tolerance);

    smoother_ = std::make_shared<rrt_core::RuckigSmoother>(dof_, 0.01);

    // collision_checker_ = std::make_shared<collision::CheckCollision>(urdf_); // only to compute path length

    RCLCPP_INFO(this->get_logger(), "RRT Node initialized. Waiting for goal...");
}

void RRTNode::run(uint8_t planner_type)
{
    bool success = false;

    auto start_time = std::chrono::high_resolution_clock::now();

    switch (planner_type)
    {
    case GoalCommand::RRT:
    {
        if (!rrt_) // construct only once
        {
            // RCLCPP_INFO(this->get_logger(), "Initializing RRT planner...");
            rrt_ = std::make_shared<rrt_core::RRT>(dof_, urdf_);
            rrt_->setJointLimits(lower_limits_, upper_limits_);
            rrt_->setStepSize(step_size);
            rrt_->setGoalBias(goal_bias);
            rrt_->setGoalTolerance(goal_tolerance);
        }

        try
        {
            rrt_->setStart(q_start_);
            rrt_->setGoal(q_goal_);
            success = rrt_->plan(max_iterations_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Planning error: %s", e.what());
            return;
        }

        break;
    }

    case GoalCommand::RRT_STAR:
    {
        if (!rrt_star_) // construct only once
        {
            // RCLCPP_INFO(this->get_logger(), "Initializing RRT* planner...");
            rrt_star_ = std::make_shared<rrt_core::RRTStar>(dof_, urdf_);
            rrt_star_->setJointLimits(lower_limits_, upper_limits_);
            rrt_star_->setStepSize(step_size);
            rrt_star_->setGoalBias(goal_bias);
            rrt_star_->setGoalTolerance(goal_tolerance);
        }

        try
        {
            rrt_star_->setStart(q_start_);
            rrt_star_->setGoal(q_goal_);
            success = rrt_star_->plan(max_iterations_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Planning error: %s", e.what());
            return;
        }

        break;
    }

    case GoalCommand::RRT_CONNECT:
    {
        // Not implemented yet — fall through to warning
        RCLCPP_WARN(this->get_logger(), "RRT-Connect (type %d) is not implemented yet. Falling back to RRT.",
                    planner_type);
        rrt_->setStart(q_start_);
        rrt_->setGoal(q_goal_);
        success = rrt_->plan(max_iterations_);
        break;
    }

    default:
    {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown planner type %d. Falling back to RRT.", planner_type);
        rrt_->setStart(q_start_);
        rrt_->setGoal(q_goal_);
        success = rrt_->plan(max_iterations_);
        break;
    }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Path found!");

        path_ = (planner_type == GoalCommand::RRT_STAR) ? rrt_star_->getPath()
                                                        : rrt_->getPath();
        // path_ = rrt_->shortcutPath(path_);
        // path_ = rrt_->interpolatePath(path_, 200); // cubic spline, doesnt work

        RCLCPP_INFO(this->get_logger(), "Total way points: %ld", path_.size());

        // auto traj = smoother_->smoothPath(path_, max_vel, max_acc, max_jerk);
        // auto traj = smoother_->timeParametrize(path_, max_vel, max_acc, max_jerk);

        // path_ = traj.positions; // smooth, jerk-limited positions

        // RCLCPP_INFO(this->get_logger(), "Trajectory duration: %.3f s, Points: %ld", traj.duration, path_.size());

        if (is_debug)
        {
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
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Planning failed!");
    }

    tcp_path.clear();

    publishTrajectory(path_);

    double length = rrt_->computePathLength(path_, tcp_path);
    publishTCPPath(tcp_path);

    RCLCPP_INFO(this->get_logger(), "Cartesian path length: %.4f meters, Planning time: %.3f ms", length, duration_ms);
}

void RRTNode::goalCallback(const GoalCommand::SharedPtr msg)
{
    if (msg->goal.size() != static_cast<size_t>(dof_))
    {
        RCLCPP_WARN(this->get_logger(), "Invalid goal size! Expected %d DOF, got %ld.", dof_, msg->goal.size());
        return;
    }

    q_goal_ = msg->goal;

    RCLCPP_INFO(this->get_logger(), "Received goal. Planner type: %d. Starting planning...", static_cast<int>(msg->planner_type));
    run(msg->planner_type);
}

void RRTNode::publishTrajectory(const std::vector<std::vector<double>> &path)
{
    if (path.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty path, not publishing trajectory.");
        return;
    }

    trajectory_msgs::msg::JointTrajectory traj;

    // traj.header.stamp = this->now();
    traj.joint_names = joint_names_;

    double time_from_start = 0.0;
    double dt = 0.2; // adjust based on speed


    for (size_t i = 0; i < path.size(); ++i)
    {
    trajectory_msgs::msg::JointTrajectoryPoint point;

        point.positions = path[i];

        // Optional (recommended if using smoother later)
        // point.velocities.resize(dof_, 0.0);
        // point.accelerations.resize(dof_, 0.0);

        point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
        time_from_start += dt;

        traj.points.push_back(point);
    }

    // trajectory_msgs::msg::JointTrajectoryPoint point;
    // point.positions = path[path.size() - 1];
    // point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    // traj.points.push_back(point);
    
    traj_pub_->publish(traj);

    RCLCPP_INFO(this->get_logger(), "Published trajectory with %ld points.", path.size());
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