#include "rrt_impl/check_collision.hpp"
#include <iostream>

namespace collision
{

    CheckCollision::CheckCollision(const std::string &urdf_path)
    {
        try
        {
            // Load kinematic model
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = pinocchio::Data(model_);

            // Load collision geometry
            pinocchio::urdf::buildGeom(
                model_,
                urdf_path,
                pinocchio::COLLISION,
                geom_model_);

            // 🔥 IMPORTANT: add collision pairs
            geom_model_.addAllCollisionPairs();

            // Remove collision pairs between bodies connected by a joint
            for (size_t i = 0; i < geom_model_.collisionPairs.size();)
            {
                const pinocchio::CollisionPair &pair = geom_model_.collisionPairs[i];

                const pinocchio::GeometryObject &go1 = geom_model_.geometryObjects[pair.first];
                const pinocchio::GeometryObject &go2 = geom_model_.geometryObjects[pair.second];

                // Get the parent joints of each geometry object
                pinocchio::JointIndex parent1 = go1.parentJoint;
                pinocchio::JointIndex parent2 = go2.parentJoint;

                // Remove if same parent joint or if one is the direct parent of the other
                bool same_parent = (parent1 == parent2);
                bool direct_parent = (model_.parents[parent1] == parent2 ||
                                      model_.parents[parent2] == parent1);

                if (same_parent || direct_parent)
                {
                    geom_model_.collisionPairs.erase(
                        geom_model_.collisionPairs.begin() + i);
                }
                else
                {
                    ++i;
                }
            }

            geom_data_ = pinocchio::GeometryData(geom_model_);

            dof_ = model_.nq;

            // ===== DEBUG =====
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

        Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), dof_);

        bool in_collision = pinocchio::computeCollisions(
            model_, data_, geom_model_, geom_data_, q_eigen, true);

        // ===== DEBUG =====
        std::cout << "[Collision] Checking q: ";
        for (double v : q)
            std::cout << v << " ";
        std::cout << std::endl;

        if (in_collision)
        {
            std::cout << "[Collision] ❌ In collision!" << std::endl;

            // 🔥 Print WHICH pair collided
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
        }
        else
        {
            std::cout << "[Collision] ✅ Collision-free" << std::endl;
        }

        std::cout << "-----------------------------------" << std::endl;

        return !in_collision;
    }

} // namespace collision