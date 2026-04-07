#include "rrt_impl/check_collision.hpp"
#include <iostream>

namespace collision
{

    CheckCollision::CheckCollision(const std::string &urdf_path)
    {
        try
        {
            // ================= FULL MODEL =================
            pinocchio::Model full_model;
            pinocchio::urdf::buildModel(urdf_path, full_model);

            // ================= LOCK FINGER JOINTS =================
            std::vector<pinocchio::JointIndex> joints_to_lock;

            auto add_if_exists = [&](const std::string &name)
            {
                if (full_model.existJointName(name))
                    joints_to_lock.push_back(full_model.getJointId(name));
            };

            add_if_exists("panda_finger_joint1");
            add_if_exists("panda_finger_joint2");

            Eigen::VectorXd q_ref = Eigen::VectorXd::Zero(full_model.nq);

            // ================= REDUCED MODEL =================
            pinocchio::buildReducedModel(full_model, joints_to_lock, q_ref, model_);
            data_ = pinocchio::Data(model_);

            // ================= GEOMETRY =================
            pinocchio::urdf::buildGeom(
                model_,
                urdf_path,
                pinocchio::COLLISION,
                geom_model_);

            geom_model_.addAllCollisionPairs();

            // ================= END EFFECTOR =================
            std::string ee_name = "panda_link7"; // safest choice

            if (!model_.existFrame(ee_name))
            {
                std::cout << "[WARN] EE frame " << ee_name
                          << " not found!" << std::endl;
                throw std::runtime_error("Invalid EE frame");
            }

            ee_frame_id_ = model_.getFrameId(ee_name);

            // ================= FILTER COLLISION PAIRS =================
            for (size_t i = 0; i < geom_model_.collisionPairs.size();)
            {
                const auto &pair = geom_model_.collisionPairs[i];

                const auto &go1 = geom_model_.geometryObjects[pair.first];
                const auto &go2 = geom_model_.geometryObjects[pair.second];

                // Remove hand/finger collisions
                bool remove_hand =
                    (go1.name.find("finger") != std::string::npos ||
                     go1.name.find("hand") != std::string::npos ||
                     go2.name.find("finger") != std::string::npos ||
                     go2.name.find("hand") != std::string::npos);

                // Remove adjacent links
                pinocchio::JointIndex parent1 = go1.parentJoint;
                pinocchio::JointIndex parent2 = go2.parentJoint;

                bool same_parent = (parent1 == parent2);

                bool direct_parent =
                    (parent1 < model_.parents.size() &&
                     parent2 < model_.parents.size() &&
                     (model_.parents[parent1] == parent2 ||
                      model_.parents[parent2] == parent1));

                if (remove_hand || same_parent || direct_parent)
                {
                    geom_model_.collisionPairs.erase(
                        geom_model_.collisionPairs.begin() + i);
                }
                else
                {
                    ++i;
                }
            }

            // ================= FINALIZE =================
            geom_data_ = pinocchio::GeometryData(geom_model_);
            dof_ = model_.nq;

            // ================= DEBUG =================
            std::cout << "===== COLLISION INIT =====" << std::endl;
            std::cout << "DOF: " << dof_ << std::endl;
            std::cout << "Geometry objects: "
                      << geom_model_.geometryObjects.size() << std::endl;
            std::cout << "Collision pairs: "
                      << geom_model_.collisionPairs.size() << std::endl;
            std::cout << "==========================" << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error: " << e.what() << std::endl;
            throw;
        }
    }

    bool CheckCollision::isStateValid(const std::vector<double> &q) const
    {
        if (static_cast<int>(q.size()) != dof_)
        {
            std::cout << "[Collision] ❌ Invalid DOF size\n";
            return false;
        }

        Eigen::VectorXd q_eigen =
            Eigen::Map<const Eigen::VectorXd>(q.data(), dof_);

        bool in_collision = pinocchio::computeCollisions(
            model_, data_, geom_model_, geom_data_, q_eigen, true);

        // Print only when collision happens (faster)
        if (in_collision)
        {
            std::cout << "[Collision] ❌ In collision!" << std::endl;

            for (size_t i = 0; i < geom_data_.collisionResults.size(); ++i)
            {
                if (geom_data_.collisionResults[i].isCollision())
                {
                    const auto &pair = geom_model_.collisionPairs[i];

                    std::cout << "  → Collision between: "
                              << geom_model_.geometryObjects[pair.first].name
                              << " AND "
                              << geom_model_.geometryObjects[pair.second].name
                              << std::endl;
                }
            }
            std::cout << "-----------------------------------" << std::endl;
        }

        return !in_collision;
    }

    double CheckCollision::computeCartesianPathLength(const std::vector<std::vector<double>> &path,
                                                      std::vector<Eigen::Vector3d> &tcp_points)
    {
        if (path.size() < 2)
            return 0.0;

        double total_length = 0.0;
        Eigen::Vector3d prev_pos;

        tcp_points.clear();
        tcp_points.reserve(path.size());

        for (size_t i = 0; i < path.size(); ++i)
        {
            if (path[i].size() != static_cast<size_t>(dof_))
            {
                throw std::runtime_error("Invalid DOF in path");
            }

            Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(path[i].data(), dof_);

            // Forward kinematics
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);

            Eigen::Vector3d curr_pos = data_.oMf[ee_frame_id_].translation();

            tcp_points.push_back(curr_pos);

            if (i > 0)
            {
                total_length += (curr_pos - prev_pos).norm();
            }

            prev_pos = curr_pos;
        }

        return total_length;
    }

} // namespace collision