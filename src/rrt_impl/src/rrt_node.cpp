#include "rrt_impl/rrt_node.hpp"
#include <sstream>

RRTNode::RRTNode() : Node("rrt_node")
{

    goal_sub_ = this->create_subscription<GoalCommand>("/goal_command", 10, std::bind(&RRTNode::goalCallback, this, std::placeholders::_1));

    scene_sub_ = create_subscription<custom_msgs::msg::SceneCommand>("/scene_command", 10,
                                                                     std::bind(&RRTNode::sceneCommandCallback, this, std::placeholders::_1));

    tcp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rrt_path", 10);
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

    scene_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/scene_objects", 10);

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

    collision_checker_ = std::make_shared<collision::CheckCollision>(urdf_);

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
            rrt_ = std::make_shared<rrt_core::RRT>(dof_, collision_checker_);
            rrt_->setJointLimits(lower_limits_, upper_limits_);
            rrt_->setStepSize(step_size);
            rrt_->setGoalBias(goal_bias);
            rrt_->setGoalTolerance(goal_tolerance);
        }
        planner_ = rrt_;
        break;
    }

    case GoalCommand::RRT_STAR:
    {
        if (!rrt_star_) // construct only once
        {
            // RCLCPP_INFO(this->get_logger(), "Initializing RRT* planner...");
            rrt_star_ = std::make_shared<rrt_core::RRTStar>(dof_, collision_checker_);
            rrt_star_->setJointLimits(lower_limits_, upper_limits_);
            rrt_star_->setStepSize(step_size);
            rrt_star_->setGoalBias(goal_bias);
            rrt_star_->setGoalTolerance(goal_tolerance);
        }
        planner_ = rrt_star_;
        break;
    }

    case GoalCommand::RRT_CONNECT:
    {
        if (!rrt_connect_) // construct only once
        {
            // RCLCPP_INFO(this->get_logger(), "Initializing RRT-Connect planner...");
            rrt_connect_ = std::make_shared<rrt_core::RRTConnect>(dof_, collision_checker_);
            rrt_connect_->setJointLimits(lower_limits_, upper_limits_);
            rrt_connect_->setStepSize(step_size);
            rrt_connect_->setGoalBias(goal_bias);
            rrt_connect_->setGoalTolerance(goal_tolerance);
        }
        planner_ = rrt_connect_;
        break;
    }

    default:
    {
        RCLCPP_WARN(this->get_logger(), "Unknown planner type %d. Falling back to RRT.", planner_type);

        if (!rrt_)
        {
            rrt_ = std::make_shared<rrt_core::RRT>(dof_, collision_checker_);
            rrt_->setJointLimits(lower_limits_, upper_limits_);
            rrt_->setStepSize(step_size);
            rrt_->setGoalBias(goal_bias);
            rrt_->setGoalTolerance(goal_tolerance);
        }

        planner_ = rrt_;
        break;
    }
    }

    if (!planner_)
    {
        RCLCPP_ERROR(this->get_logger(), "Planner not initialized!");
        return;
    }

    try
    {
        planner_->setStart(q_start_);
        planner_->setGoal(q_goal_);
        // testIK();

        success = planner_->plan(max_iterations_);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Planning error: %s", e.what());
        return;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    if (success)
    {
        // std::cout << "planner_: " << planner_.get() << std::endl;
        // std::cout << "rrt_: " << rrt_.get() << std::endl;
        // std::cout << "rrt_star_: " << rrt_star_.get() << std::endl;
        // std::cout << "rrt_connect_: " << rrt_connect_.get() << std::endl;

        RCLCPP_INFO(this->get_logger(), "Path found!");

        path_ = planner_->getPath();

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

    double length = collision_checker_->computeCartesianPathLength(path_, tcp_path);
    publishTCPPath(tcp_path);

    RCLCPP_INFO(this->get_logger(), "Cartesian path length: %.4f meters, Planning time: %.3f ms", length, duration_ms);
}

void RRTNode::testIK()
{
    geometry_msgs::msg::Pose test_pose;
    test_pose.position.x = 0.30915;
    test_pose.position.y = 0.0;
    test_pose.position.z = 0.69374;

    test_pose.orientation.w = 0.0019;
    test_pose.orientation.x = 0.92485;
    test_pose.orientation.y = -0.38015;
    test_pose.orientation.z = 0.00185;

    std::vector<double> q_ik;

    if (collision_checker_->getIK(test_pose, q_ik))
    {
        RCLCPP_INFO(this->get_logger(), "IK solution found:");
        for (size_t i = 0; i < q_ik.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "  Joint %ld: %.4f", i, q_ik[i]);
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "IK solution not found for the given pose.");
    }
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

    // ---- Step 1: Compute TCP path ----
    std::vector<Eigen::Vector3d> tcp_path_local;

    if (!planner_)
    {
        RCLCPP_ERROR(this->get_logger(), "Planner is null in publishTrajectory!");
        return;
    }

    double total_length = collision_checker_->computeCartesianPathLength(path, tcp_path_local);
    if (total_length <= 1e-6)
    {
        RCLCPP_WARN(this->get_logger(), "Path length too small.");
        return;
    }

    // ---- Step 2: Remove duplicate / tiny segments (IMPORTANT for RRT*) ----
    std::vector<std::vector<double>> clean_path; // if consecutive points have same timestamps, the controller may ignore them,
    std::vector<Eigen::Vector3d> clean_tcp;

    const double min_dist = 1e-5;

    if (tcp_path_local.size() != path.size())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "TCP path mismatch! tcp: %ld vs path: %ld",
                     tcp_path_local.size(), path.size());
        return;
    }

    clean_path.push_back(path[0]);
    clean_tcp.push_back(tcp_path_local[0]);

    for (size_t i = 1; i < path.size(); ++i)
    {
        double dist = (tcp_path_local[i] - tcp_path_local[i - 1]).norm();

        if (dist > min_dist)
        {
            clean_path.push_back(path[i]);
            clean_tcp.push_back(tcp_path_local[i]);
        }
    }

    if (clean_path.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Path collapsed after filtering.");
        return;
    }

    // ---- Step 3: Recompute total length after cleanup ----
    double clean_length = 0.0;
    for (size_t i = 1; i < clean_tcp.size(); ++i)
    {
        clean_length += (clean_tcp[i] - clean_tcp[i - 1]).norm();
    }

    // ---- Step 4: Define Cartesian speed ----
    double cartesian_speed = 0.2; // m/s
    double total_time = clean_length / cartesian_speed;

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names_;

    double accumulated_time = 1e-3; // small epsilon (avoid 0)

    const double min_dt = 1e-3; // ensure strictly increasing

    for (size_t i = 0; i < clean_path.size(); ++i)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = clean_path[i];

        // Optional (recommended)
        point.velocities.resize(dof_, 0.0);

        if (i > 0)
        {
            double segment_length = (clean_tcp[i] - clean_tcp[i - 1]).norm();

            double segment_time = (segment_length / std::max(clean_length, 1e-6)) * total_time;

            // 🔥 Critical fix
            segment_time = std::max(segment_time, min_dt);

            accumulated_time += segment_time;
        }

        // 🔥 Ensure last point exactly matches total_time
        if (i == clean_path.size() - 1)
        {
            accumulated_time = std::max(accumulated_time, total_time);
        }

        point.time_from_start = rclcpp::Duration::from_seconds(accumulated_time);

        traj.points.push_back(point);
    }

    traj_pub_->publish(traj);

    RCLCPP_INFO(this->get_logger(),
                "Published trajectory | Length: %.3f m | Time: %.3f s | Points: %ld -> %ld",
                clean_length, total_time, path.size(), clean_path.size());
}

// void RRTNode::publishTrajectory(const std::vector<std::vector<double>> &path)
// {
//     if (path.empty())
//     {
//         RCLCPP_WARN(this->get_logger(), "Empty path, not publishing trajectory.");
//         return;
//     }

//     trajectory_msgs::msg::JointTrajectory traj;

//     // traj.header.stamp = this->now();
//     traj.joint_names = joint_names_;

//     double time_from_start = 0.0;
//     double dt = 0.2; // adjust based on speed

//     for (size_t i = 0; i < path.size(); ++i)
//     {
//         trajectory_msgs::msg::JointTrajectoryPoint point;

//         point.positions = path[i];

//         // Optional (recommended if using smoother later)
//         // point.velocities.resize(dof_, 0.0);
//         // point.accelerations.resize(dof_, 0.0);

//         point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
//         time_from_start += dt;

//         traj.points.push_back(point);
//     }

//     // trajectory_msgs::msg::JointTrajectoryPoint point;
//     // point.positions = path[path.size() - 1];
//     // point.time_from_start = rclcpp::Duration::from_seconds(2.0);
//     // traj.points.push_back(point);

//     traj_pub_->publish(traj);

//     RCLCPP_INFO(this->get_logger(), "Published trajectory with %ld points.", path.size());
// }

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

void RRTNode::sceneCommandCallback(const custom_msgs::msg::SceneCommand::SharedPtr msg)
{
    using Cmd = custom_msgs::msg::SceneCommand;

    switch (msg->action)
    {
    case Cmd::ADD_PRIMITIVE:
    {
        const auto &p = msg->primitive;

        // ===== PRINT BASIC INFO =====
        RCLCPP_INFO(this->get_logger(), "Action: ADD_PRIMITIVE");
        RCLCPP_INFO(this->get_logger(), "Name  : %s", p.name.c_str());
        RCLCPP_INFO(this->get_logger(), "Type  : %d", p.type);

        // ===== PRINT PARAMS =====
        std::stringstream ss;
        ss << "Params: ";
        for (auto val : p.params)
            ss << val << " ";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(p.pose.position.x,
                                             p.pose.position.y,
                                             p.pose.position.z);
        Eigen::Quaterniond q(p.pose.orientation.w,
                             p.pose.orientation.x,
                             p.pose.orientation.y,
                             p.pose.orientation.z);
        pose.linear() = q.toRotationMatrix();

        collision_checker_->addPrimitive(p.name, static_cast<collision::PrimitiveType>(p.type),
                                         std::vector<double>(p.params.begin(), p.params.end()), pose);
        break;
    }
    case Cmd::ADD_MESH:
    {
        const auto &m = msg->mesh;

        std::vector<Eigen::Vector3d> verts;
        for (const auto &pt : m.vertices)
            verts.emplace_back(pt.x, pt.y, pt.z);

        // unflatten: [v0,v1,v2, v0,v1,v2, ...]
        std::vector<std::array<int, 3>> tris;
        for (size_t i = 0; i + 2 < m.triangles.size(); i += 3)
            tris.push_back({m.triangles[i], m.triangles[i + 1], m.triangles[i + 2]});

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(m.pose.position.x,
                                             m.pose.position.y,
                                             m.pose.position.z);
        Eigen::Quaterniond q(m.pose.orientation.w,
                             m.pose.orientation.x,
                             m.pose.orientation.y,
                             m.pose.orientation.z);
        pose.linear() = q.toRotationMatrix();

        collision_checker_->addMesh(m.name, verts, tris, pose);
        break;
    }
    case Cmd::REMOVE:
        collision_checker_->removeSceneObject(msg->remove_name);
        break;

    case Cmd::CLEAR:
        collision_checker_->clearSceneObjects();
        break;

    default:
        RCLCPP_WARN(get_logger(), "Unknown scene command action: %d", msg->action);
    }

    publishSceneMarkers();
}

void RRTNode::publishSceneMarkers()
{
    auto objects = collision_checker_->getSceneObjects();

    visualization_msgs::msg::MarkerArray marker_array;

    // First publish a DELETE_ALL to clear old markers
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_all.header.frame_id = "world";
    delete_all.header.stamp = this->now();
    marker_array.markers.push_back(delete_all);
    scene_marker_pub_->publish(marker_array);
    marker_array.markers.clear();

    int id = 0;
    for (const auto &obj : objects)
    {
        // Skip ground plane — it clutters the view
        if (obj.name == "ground_plane")
        {
            ++id;
            continue;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = "scene_objects";
        marker.id = id++;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // ── Pose ──────────────────────────────────────────────────────────
        marker.pose.position.x = obj.position.x();
        marker.pose.position.y = obj.position.y();
        marker.pose.position.z = obj.position.z();
        marker.pose.orientation.x = obj.orientation.x();
        marker.pose.orientation.y = obj.orientation.y();
        marker.pose.orientation.z = obj.orientation.z();
        marker.pose.orientation.w = obj.orientation.w();

        // ── Color — semi-transparent red ──────────────────────────────────
        marker.color.r = 1.0f;
        marker.color.g = 0.2f;
        marker.color.b = 0.2f;
        marker.color.a = 0.6f;

        // ── Shape + scale ─────────────────────────────────────────────────
        if (obj.type == "sphere" && obj.dims.size() >= 1)
        {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            double r = obj.dims[0];
            marker.scale.x = r * 2.0;
            marker.scale.y = r * 2.0;
            marker.scale.z = r * 2.0;
        }
        else if (obj.type == "box" && obj.dims.size() >= 3)
        {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = obj.dims[0];
            marker.scale.y = obj.dims[1];
            marker.scale.z = obj.dims[2];
        }
        else if (obj.type == "cylinder" && obj.dims.size() >= 2)
        {
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.scale.x = obj.dims[0] * 2.0; // diameter
            marker.scale.y = obj.dims[0] * 2.0;
            marker.scale.z = obj.dims[1]; // full length
        }
        else if (obj.type == "capsule" && obj.dims.size() >= 2)
        {
            // RViz has no capsule — approximate with cylinder
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.scale.x = obj.dims[0] * 2.0;
            marker.scale.y = obj.dims[0] * 2.0;
            marker.scale.z = obj.dims[1] + obj.dims[0] * 2.0; // shaft + caps
        }
        else if (obj.type == "cone" && obj.dims.size() >= 2)
        {
            // RViz has no cone — use a flattened sphere as fallback
            // or use ARROW which looks cone-like
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.scale.x = obj.dims[1];       // shaft length = height
            marker.scale.y = obj.dims[0] * 2.0; // head diameter = base diameter
            marker.scale.z = obj.dims[0] * 2.0;
        }
        else if (obj.type == "mesh" && obj.dims.size() >= 3)
        {
            // Show AABB as a cube for meshes
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = obj.dims[0];
            marker.scale.y = obj.dims[1];
            marker.scale.z = obj.dims[2];
            marker.color.r = 0.2f;
            marker.color.g = 0.6f;
            marker.color.b = 1.0f;
            marker.color.a = 0.5f;
        }
        else
        {
            continue; // skip unknown
        }

        marker_array.markers.push_back(marker);
    }

    scene_marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "[Scene] Published %zu markers to RViz",
                marker_array.markers.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RRTNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}