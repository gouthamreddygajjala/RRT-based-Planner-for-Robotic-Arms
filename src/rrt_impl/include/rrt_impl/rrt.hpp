#pragma once

#include <vector>
#include <random>

#include <limits>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "rrt_impl/collision.hpp"
#include "kd_tree.hpp"
#include <geometry_msgs/msg/pose.hpp>

namespace rrt_core
{

    struct RRTNode
    {
        std::vector<double> q;
        int parent_index;
        // std::vector<int> children;
    };

    using PrimitiveType = collision::PrimitiveType;

    class RRT
    {
    public:
        virtual ~RRT() = default;

        explicit RRT(size_t dof, const std::string &urdf_path);

        explicit RRT(size_t dof, std::shared_ptr<collision::CheckCollision> collision);

        void setStart(const std::vector<double> &q);
        void setGoal(const std::vector<double> &q);

        void setJointLimits(const std::vector<double> &lower,
                            const std::vector<double> &upper);

        void setStepSize(double step);
        void setGoalBias(double bias);
        void setGoalTolerance(double tol);

        void reserve(size_t n);

        virtual bool plan(int max_iter);
        virtual std::vector<std::vector<double>> getPath() const;

        const std::vector<RRTNode> &getNodes() const;

        std::vector<std::vector<double>> shortcutPath(const std::vector<std::vector<double>> &path);
        std::vector<std::vector<double>> interpolatePath(const std::vector<std::vector<double>> &path, int num_points);

        size_t dof_;
        std::vector<RRTNode> nodes_;

        double step_size_{0.06}; // 0.1
        double goal_bias_{0.1};
        double goal_tolerance_{0.1};

        int goal_index_{-1};

        std::vector<double> q_start_;
        std::vector<double> q_goal_;

        std::vector<double> lower_limits_;
        std::vector<double> upper_limits_;

        std::vector<double> sampleRandomConfig();
        int getNearestNode(const std::vector<double> &q);
        std::vector<double> steer(const std::vector<double> &q_near,
                                  const std::vector<double> &q_rand);

        double distance(const std::vector<double> &a,
                        const std::vector<double> &b) const;

        bool isCollisionFree(const std::vector<double> &q) const;
        bool isEdgeCollisionFree(const std::vector<double> &q1,
                                 const std::vector<double> &q2) const;

        void validateConfig(const std::vector<double> &q) const;

        double computePathLength(const std::vector<std::vector<double>> &path, std::vector<Eigen::Vector3d> &tcp_points);

        double checkIK(const geometry_msgs::msg::Pose &pose, std::vector<double> &q_out);

        void addPrimitive(const std::string &name, collision::PrimitiveType type,
                          const std::vector<double> &params, const Eigen::Isometry3d &pose = Eigen::Isometry3d::Identity());

        void addMesh(const std::string &name, const std::vector<Eigen::Vector3d> &vertices,
                     const std::vector<std::array<int, 3>> &triangles, const Eigen::Isometry3d &pose = Eigen::Isometry3d::Identity());

        void clearSceneObjects();
        bool removeSceneObject(const std::string &name);

    private:
        std::mt19937 rng_;

        std::shared_ptr<collision::CheckCollision> collision_checker_;
        std::vector<Eigen::Vector3d> tcp_points;

    public:
        KDTree kd_tree_;
    };

} // namespace rrt_core