#include "rrt_impl/collision.hpp"

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

            robot_geom_model_ = geom_model_;  // ← save clean robot-only copy
            robot_geom_count_ = geom_model_.geometryObjects.size();

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

            // ================= GROUND PLANE =================
            {
                Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

                // Place ground at z = 0 (adjust if robot base is offset)
                pose.translation() = Eigen::Vector3d(0.0, 0.0, -0.01); // slight offset below 0

                // Large flat box: 10m x 10m x 2cm
                std::vector<double> params = {10.0, 10.0, 0.02};

                addPrimitive("ground_plane", PrimitiveType::BOX, params, pose);

                std::cout << "[Scene] 🌍 Ground plane added\n";
            }

            // ================= ALLOW BASE ↔ GROUND =================
            {
                pinocchio::GeomIndex ground_idx = geom_model_.geometryObjects.size() - 1;

                for (size_t i = 0; i < geom_model_.collisionPairs.size();)
                {
                    const auto &pair = geom_model_.collisionPairs[i];

                    const auto &go1 = geom_model_.geometryObjects[pair.first];
                    const auto &go2 = geom_model_.geometryObjects[pair.second];

                    bool is_ground_pair = (pair.first == ground_idx || pair.second == ground_idx);

                    bool is_base_link = (go1.name.find("panda_link0") != std::string::npos ||
                                         go2.name.find("panda_link0") != std::string::npos);

                    if (is_ground_pair && is_base_link)
                    {
                        std::cout << "[Scene] ⚙️ Allowing ground ↔ base collision: "
                                  << go1.name << " <-> " << go2.name << std::endl;

                        geom_model_.collisionPairs.erase(
                            geom_model_.collisionPairs.begin() + i);
                    }
                    else
                    {
                        ++i;
                    }
                }

                // Refresh geom data after modification
                geom_data_ = pinocchio::GeometryData(geom_model_);
            }
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

    void CheckCollision::addPrimitive(const std::string &name, PrimitiveType type,
                                      const std::vector<double> &params, const Eigen::Isometry3d &pose)
    {
        std::shared_ptr<coal::CollisionGeometry> geom;

        switch (type)
        {
        case PrimitiveType::SPHERE:
            if (params.size() < 1)
                throw std::invalid_argument("SPHERE needs [radius]");
            geom = std::make_shared<coal::Sphere>(params[0]);
            break;

        case PrimitiveType::CYLINDER:
            if (params.size() < 2)
                throw std::invalid_argument("CYLINDER needs [radius, length]");
            geom = std::make_shared<coal::Cylinder>(params[0], params[1] * 0.5);
            break;

        case PrimitiveType::BOX:
            if (params.size() < 3)
                throw std::invalid_argument("BOX needs [lx, ly, lz]");
            geom = std::make_shared<coal::Box>(params[0] * 0.5, params[1] * 0.5, params[2] * 0.5);
            break;

        case PrimitiveType::CAPSULE:
            if (params.size() < 2)
                throw std::invalid_argument("CAPSULE needs [radius, length]");
            geom = std::make_shared<coal::Capsule>(params[0], params[1] * 0.5);
            break;

        case PrimitiveType::CONE:
            if (params.size() < 2)
                throw std::invalid_argument("CONE needs [radius, height]");
            geom = std::make_shared<coal::Cone>(params[0], params[1] * 0.5);
            break;

        default:
            throw std::invalid_argument("Unknown PrimitiveType");
        }

        registerSceneGeometry(name, geom, pose);
    }

    void CheckCollision::addMesh(const std::string &name, const std::vector<Eigen::Vector3d> &vertices,
                                 const std::vector<std::array<int, 3>> &triangles, const Eigen::Isometry3d &pose)
    {
        using BVHModel = coal::BVHModel<coal::OBBRSS>;
        auto mesh = std::make_shared<BVHModel>();

        mesh->beginModel(static_cast<int>(triangles.size()),
                         static_cast<int>(vertices.size()));

        std::vector<coal::Vec3f> pts;
        pts.reserve(vertices.size());
        for (const auto &v : vertices)
            pts.emplace_back(static_cast<float>(v.x()),
                             static_cast<float>(v.y()),
                             static_cast<float>(v.z()));

        std::vector<coal::Triangle> tris;
        tris.reserve(triangles.size());
        for (const auto &t : triangles)
            tris.emplace_back(t[0], t[1], t[2]);

        mesh->addSubModel(pts, tris);
        mesh->endModel();
        mesh->computeLocalAABB();

        registerSceneGeometry(name, mesh, pose);
    }

    void CheckCollision::registerSceneGeometry(
        const std::string &name,
        const std::shared_ptr<coal::CollisionGeometry> &geom,
        const Eigen::Isometry3d &pose)
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);

        pinocchio::SE3 placement(pose.rotation(), pose.translation());
        pinocchio::GeometryObject obj(name, 0, geom, placement);

        // Store the object for rebuild capability
        scene_objects_.push_back(obj);

        // Rebuild everything cleanly
        rebuildGeomModel();

        std::cout << "[Scene] ✅ Added '" << name
                  << "' | pairs: " << geom_model_.collisionPairs.size() << "\n";
    }

    // ── Internal: rebuild geom_model_ from robot base + current scene objects ───
    void CheckCollision::rebuildGeomModel()
    {
        // 1. Restore robot-only geometry model
        geom_model_ = robot_geom_model_; // clean copy of original robot geom

        // 2. Re-add all surviving scene objects
        for (auto &obj : scene_objects_)
        {
            pinocchio::GeomIndex new_idx = geom_model_.addGeometryObject(obj);

            // Pair against every robot link (skip finger/hand)
            for (std::size_t i = 0; i < robot_geom_count_; ++i)
            {
                const auto &go = geom_model_.geometryObjects[i];
                if (go.name.find("finger") != std::string::npos ||
                    go.name.find("hand") != std::string::npos)
                    continue;
                geom_model_.addCollisionPair(pinocchio::CollisionPair(i, new_idx));
            }

            // Pair against previously re-added scene objects
            for (std::size_t i = robot_geom_count_;
                 i < geom_model_.geometryObjects.size() - 1; ++i)
            {
                geom_model_.addCollisionPair(
                    pinocchio::CollisionPair(i, new_idx));
            }
        }

        geom_data_ = pinocchio::GeometryData(geom_model_);

        // Rebuild index list
        scene_object_indices_.clear();
        for (std::size_t i = robot_geom_count_;
             i < geom_model_.geometryObjects.size(); ++i)
            scene_object_indices_.push_back(i);
    }

    bool CheckCollision::removeSceneObject(const std::string &name)
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);

        // Find in scene_objects_ list by name
        auto it = std::find_if(scene_objects_.begin(), scene_objects_.end(),
                               [&name](const pinocchio::GeometryObject &obj)
                               { return obj.name == name; });

        if (it == scene_objects_.end())
        {
            std::cout << "[Scene] ⚠️  Object '" << name << "' not found.\n";
            return false;
        }

        scene_objects_.erase(it);
        rebuildGeomModel();

        std::cout << "[Scene] 🗑️  Removed '" << name << "'\n";
        return true;
    }

    void CheckCollision::clearSceneObjects()
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);
        scene_objects_.clear();
        rebuildGeomModel();
        std::cout << "[Scene] 🗑️  Cleared all scene objects.\n";
    }

    std::vector<CheckCollision::SceneObjectInfo> CheckCollision::getSceneObjects() const
    {
        std::vector<SceneObjectInfo> result;
        result.reserve(scene_object_indices_.size());

        for (pinocchio::GeomIndex idx : scene_object_indices_)
        {
            const auto &go = geom_model_.geometryObjects[idx];
            SceneObjectInfo info;

            info.name = go.name;

            // ── Pose ────────────────────────────────────────────────────────────
            info.position = go.placement.translation();
            info.orientation = Eigen::Quaterniond(go.placement.rotation());

            // ── Geometry type + dimensions ───────────────────────────────────────
            const auto &geom = go.geometry;

            switch (geom->getNodeType())
            {
            case coal::GEOM_SPHERE:
            {
                const auto *s = dynamic_cast<const coal::Sphere *>(geom.get());
                info.type = "sphere";
                info.dims = {s->radius}; // [radius]
                break;
            }
            case coal::GEOM_CYLINDER:
            {
                const auto *c = dynamic_cast<const coal::Cylinder *>(geom.get());
                info.type = "cylinder";
                info.dims = {c->radius, c->halfLength * 2.0}; // [radius, full_length]
                break;
            }
            case coal::GEOM_BOX:
            {
                const auto *b = dynamic_cast<const coal::Box *>(geom.get());
                info.type = "box";
                info.dims = {b->halfSide[0] * 2.0,
                             b->halfSide[1] * 2.0,
                             b->halfSide[2] * 2.0}; // [lx, ly, lz]
                break;
            }
            case coal::GEOM_CAPSULE:
            {
                const auto *c = dynamic_cast<const coal::Capsule *>(geom.get());
                info.type = "capsule";
                info.dims = {c->radius, c->halfLength * 2.0}; // [radius, full_shaft_length]
                break;
            }
            case coal::GEOM_CONE:
            {
                const auto *c = dynamic_cast<const coal::Cone *>(geom.get());
                info.type = "cone";
                info.dims = {c->radius, c->halfLength * 2.0}; // [radius, full_height]
                break;
            }
            case coal::BV_OBBRSS:
            {
                // BVH = mesh — report AABB extents as a rough size indicator
                const auto *bvh = dynamic_cast<const coal::BVHModelBase *>(geom.get());
                if (bvh)
                {
                    coal::Vec3f aabb_min = bvh->aabb_local.min_;
                    coal::Vec3f aabb_max = bvh->aabb_local.max_;
                    info.type = "mesh";
                    info.dims = {
                        static_cast<double>(aabb_max[0] - aabb_min[0]), // x extent
                        static_cast<double>(aabb_max[1] - aabb_min[1]), // y extent
                        static_cast<double>(aabb_max[2] - aabb_min[2])  // z extent
                    };
                }
                else
                {
                    info.type = "mesh";
                    info.dims = {};
                }
                break;
            }
            default:
                info.type = "unknown";
                info.dims = {};
                break;
            }

            result.push_back(std::move(info));
        }

        return result;
    }

    void CheckCollision::printSceneObjects() const
    {
        auto objects = getSceneObjects();

        if (objects.empty())
        {
            std::cout << "[Scene] No objects in scene.\n";
            return;
        }

        std::cout << "===== SCENE OBJECTS (" << objects.size() << ") =====\n";
        for (const auto &obj : objects)
        {
            std::cout << "  Name : " << obj.name << "\n"
                      << "  Type : " << obj.type << "\n"
                      << "  Dims : [";
            for (size_t i = 0; i < obj.dims.size(); ++i)
                std::cout << obj.dims[i] << (i + 1 < obj.dims.size() ? ", " : "");
            std::cout << "]\n"
                      << "  Pos  : ["
                      << obj.position.x() << ", "
                      << obj.position.y() << ", "
                      << obj.position.z() << "]\n"
                      << "  Quat : [x=" << obj.orientation.x()
                      << " y=" << obj.orientation.y()
                      << " z=" << obj.orientation.z()
                      << " w=" << obj.orientation.w() << "]\n"
                      << "  --------------------------------\n";
        }
        std::cout << "=============================\n";
    }

} // namespace collision