#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <mutex>

#define COAL_DISABLE_HPP_FCL_WARNINGS

// Pinocchio core
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>

// Pinocchio 3 collision API (replaces hpp/fcl direct include)
#include <pinocchio/collision/collision.hpp>

#include <coal/shape/geometric_shapes.h>
#include <coal/BVH/BVH_model.h>

#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>

namespace collision
{

    enum class PrimitiveType
    {
        SPHERE,   // params: [radius]
        CYLINDER, // params: [radius, full_length]
        BOX,      // params: [lx, ly, lz]
        CAPSULE,  // params: [radius, full_shaft_length]
        CONE,     // params: [radius, full_height]
        MESH      // params: [ignored, use actual mesh data]
    };

    class CheckCollision
    {
    public:
        struct SceneObjectInfo
        {
            std::string name;
            PrimitiveType type;         // "sphere", "box", "cylinder", "capsule", "cone", "mesh", "unknown"
            std::vector<double> params; // shape-specific dimensions (full lengths, not half)
            Eigen::Vector3d position;
            Eigen::Quaterniond orientation;

            std::shared_ptr<coal::CollisionGeometry> mesh_geom;
        };

        explicit CheckCollision(const std::string &urdf_path);
        bool isStateValid(const std::vector<double> &q) const;
        double computeCartesianPathLength(const std::vector<std::vector<double>> &path, std::vector<Eigen::Vector3d> &tcp_points);

        bool getIK(const geometry_msgs::msg::Pose &pose, std::vector<double> &q_out);

        void addPrimitive(const std::string &name, PrimitiveType type,
                          const std::vector<double> &params, const Eigen::Isometry3d &pose = Eigen::Isometry3d::Identity());

        void addMesh(const std::string &name, const std::vector<Eigen::Vector3d> &vertices,
                     const std::vector<std::array<int, 3>> &triangles, const Eigen::Isometry3d &pose = Eigen::Isometry3d::Identity());

        bool removeSceneObject(const std::string &name);
        void clearSceneObjects();

        std::vector<SceneObjectInfo> getSceneObjects() const;
        void printSceneObjects() const;

    private:
        pinocchio::Model model_;
        mutable pinocchio::Data data_;

        pinocchio::GeometryModel robot_geom_model_; // ← pristine robot-only copy

        pinocchio::GeometryModel geom_model_;
        mutable pinocchio::GeometryData geom_data_;

        pinocchio::FrameIndex ee_frame_id_;

        int dof_;

        std::size_t robot_geom_count_{0};
        std::vector<pinocchio::GeomIndex> scene_object_indices_;
        std::vector<SceneObjectInfo> scene_objects_;

        mutable std::mutex scene_mutex_;

        void registerSceneGeometry(const std::string &name, PrimitiveType type,
                                   const std::vector<double> &params, const Eigen::Isometry3d &pose);

        void rebuildGeomModel();
    };
}