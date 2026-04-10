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

        Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), dof_);

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

    bool CheckCollision::getIK(const geometry_msgs::msg::Pose &pose, std::vector<double> &q_out)
    {
        const int max_iters = 200;
        const double eps = 1e-4;
        const double lambda = 1e-3;
        const double max_step = 0.5;

        // --- Convert Pose → Eigen ---
        Eigen::Vector3d target_pos(pose.position.x,
                                   pose.position.y,
                                   pose.position.z);

        Eigen::Quaterniond q_target(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z);

        Eigen::Matrix3d target_rot = q_target.toRotationMatrix();

        // --- Initial guess ---
        Eigen::VectorXd q = Eigen::VectorXd::Zero(dof_);

        for (int iter = 0; iter < max_iters; ++iter)
        {
            // FK
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);

            const auto &frame = data_.oMf[ee_frame_id_];

            Eigen::Vector3d curr_pos = frame.translation();
            Eigen::Matrix3d curr_rot = frame.rotation();

            // --- Position error ---
            Eigen::Vector3d pos_err = target_pos - curr_pos;

            // --- Orientation error ---
            Eigen::Matrix3d R_err = target_rot * curr_rot.transpose();
            Eigen::AngleAxisd aa(R_err);
            Eigen::Vector3d rot_err = aa.axis() * aa.angle();

            // --- Combined error ---
            Eigen::Matrix<double, 6, 1> error;
            error.head<3>() = pos_err;
            error.tail<3>() = rot_err;

            if (error.norm() < eps)
            {
                // ✅ Only check collision on the FINAL solution
                std::vector<double> q_vec(q.data(), q.data() + q.size());
                if (!isStateValid(q_vec))
                {
                    std::cout << "[IK] ✓ Converged but solution is in collision\n";
                    return false;
                }

                q_out.assign(q.data(), q.data() + q.size());
                std::cout << "[IK] ✓ Success (collision-free)\n";
                return true;
            }

            // --- Jacobian ---
            Eigen::MatrixXd J(6, dof_);
            pinocchio::computeFrameJacobian(
                model_, data_, q,
                ee_frame_id_,
                pinocchio::LOCAL_WORLD_ALIGNED,
                J);

            // --- Damped Least Squares ---
            Eigen::MatrixXd JJt = J * J.transpose();
            Eigen::MatrixXd damping =
                lambda * lambda * Eigen::MatrixXd::Identity(6, 6);

            Eigen::VectorXd dq =
                J.transpose() * (JJt + damping).ldlt().solve(error);

            // --- Step size limiting ---
            double alpha = 1.0;
            if (dq.norm() > max_step)
            {
                alpha = max_step / dq.norm();
            }

            // --- Update ---
            q += alpha * dq;

            // --- Clamp to joint limits ---
            for (int i = 0; i < dof_; ++i)
            {
                q[i] = std::clamp(q[i],
                                  model_.lowerPositionLimit[i],
                                  model_.upperPositionLimit[i]);
            }
        }

        std::cout << "[IK] ❌ Failed to converge\n";
        return false;
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