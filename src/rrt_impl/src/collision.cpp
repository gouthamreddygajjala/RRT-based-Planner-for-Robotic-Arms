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
            pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_);

            geom_model_.addAllCollisionPairs();

            // ================= END EFFECTOR =================
            std::string ee_name = "panda_hand"; // safest choice

            if (!model_.existFrame(ee_name))
            {
                std::cout << "[WARN] EE frame " << ee_name << " not found!" << std::endl;
                throw std::runtime_error("Invalid EE frame");
            }

            ee_frame_id_ = model_.getFrameId(ee_name);

            // ================= FILTER COLLISION PAIRS =================
            std::vector<pinocchio::CollisionPair> filtered;
            filtered.reserve(geom_model_.collisionPairs.size()); // avoid reallocations

            // Helper lambdas
            auto isFinger = [](const std::string &name)
            {
                return name.find("finger") != std::string::npos;
            };

            auto isHand = [](const std::string &name)
            {
                return name.find("hand") != std::string::npos;
            };

            for (const auto &pair : geom_model_.collisionPairs)
            {
                const auto &go1 = geom_model_.geometryObjects[pair.first];
                const auto &go2 = geom_model_.geometryObjects[pair.second];

                // ---------------- HAND ↔ FINGER FILTER ----------------
                bool remove_hand_finger = (isHand(go1.name) && isFinger(go2.name)) ||
                                          (isHand(go2.name) && isFinger(go1.name));

                // ---------------- ADJACENT LINK FILTER ----------------
                pinocchio::JointIndex parent1 = go1.parentJoint;
                pinocchio::JointIndex parent2 = go2.parentJoint;

                bool same_parent = (parent1 == parent2);

                bool direct_parent = false;
                if (parent1 < model_.parents.size() && parent2 < model_.parents.size())
                {
                    auto p1 = model_.parents[parent1];
                    auto p2 = model_.parents[parent2];

                    direct_parent = (p1 == parent2 || p2 == parent1);
                }

                // ---------------- FINAL DECISION ----------------
                if (!(remove_hand_finger || same_parent || direct_parent))
                {
                    filtered.push_back(pair); // keep valid pair
                }
            }

            // Replace with filtered pairs
            geom_model_.collisionPairs = std::move(filtered);

            robot_geom_model_ = geom_model_; // ← save clean robot-only copy
            robot_geom_count_ = geom_model_.geometryObjects.size();

            // ================= FINALIZE =================
            geom_data_ = pinocchio::GeometryData(geom_model_);
            dof_ = model_.nq;

            // ================= DEBUG =================
            std::cout << "===== COLLISION INIT =====" << std::endl;
            std::cout << "DOF: " << dof_ << std::endl;
            std::cout << "Geometry objects: " << geom_model_.geometryObjects.size() << std::endl;
            std::cout << "Collision pairs: " << geom_model_.collisionPairs.size() << std::endl;
            std::cout << "==========================" << std::endl;

            // ================= GROUND PLANE =================
            {
                Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

                // Place ground at z = 0 (adjust if robot base is offset)
                pose.translation() = Eigen::Vector3d(0.0, 0.0, -0.01); // slight offset below 0

                // Large flat box: 10m x 10m x 2cm
                std::vector<double> params = {10.0, 10.0, 0.002};

                addPrimitive("ground_plane", PrimitiveType::BOX, params, pose);

                std::cout << "[Scene] 🌍 Ground plane added\n";
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

    void CheckCollision::addPrimitive(const std::string &name, PrimitiveType type, const std::vector<double> &params,
                                      const Eigen::Isometry3d &pose)
    {

        std::cout << "[addPrimitive] Type: " << static_cast<int>(type) << " | Params: [";

        for (size_t i = 0; i < params.size(); ++i)
            std::cout << params[i] << (i + 1 < params.size() ? ", " : "");

        std::cout << "]\n";

        switch (type)
        {
        case PrimitiveType::SPHERE:
            if (params.size() < 1)
                throw std::invalid_argument("SPHERE needs [radius]");
            break;

        case PrimitiveType::CYLINDER:
            if (params.size() < 2)
                throw std::invalid_argument("CYLINDER needs [radius, length]");
            break;

        case PrimitiveType::BOX:
            if (params.size() < 3)
                throw std::invalid_argument("BOX needs [lx, ly, lz]");
            break;

        case PrimitiveType::CAPSULE:
            if (params.size() < 2)
                throw std::invalid_argument("CAPSULE needs [radius, length]");
            break;

        case PrimitiveType::CONE:
            if (params.size() < 2)
                throw std::invalid_argument("CONE needs [radius, height]");
            break;

        default:
            throw std::invalid_argument("Unknown PrimitiveType");
        }

        registerSceneGeometry(name, type, params, pose);
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

        // registerSceneGeometry(name, mesh, pose);
        // registerSceneGeometry(name, PrimitiveType::BOX, {0, 0, 0}, pose);

        // Store the mesh geometry directly
        SceneObjectInfo info;
        info.name = name;
        info.type = PrimitiveType::MESH;
        info.mesh_geom = mesh; // Store the built mesh
        info.position = pose.translation();
        info.orientation = Eigen::Quaterniond(pose.rotation());

        scene_objects_.push_back(info);
        rebuildGeomModel();
    }

    void CheckCollision::registerSceneGeometry(const std::string &name, PrimitiveType type,
                                               const std::vector<double> &params, const Eigen::Isometry3d &pose)
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);

        SceneObjectInfo info;
        info.name = name;
        info.type = type;
        info.params = params;

        info.position = pose.translation();
        info.orientation = Eigen::Quaterniond(pose.rotation());

        scene_objects_.push_back(info);

        rebuildGeomModel();

        std::cout << "[Scene] ✅ Added '" << name << "' | pairs: " << geom_model_.collisionPairs.size() << "\n";
    }

    void CheckCollision::rebuildGeomModel()
    {
        // 1. Restore robot-only geometry
        geom_model_ = robot_geom_model_;

        // 2. Re-add ALL scene objects with fresh geometry
        for (const auto &obj : scene_objects_) // ← now SceneObjectInfo
        {
            std::shared_ptr<coal::CollisionGeometry> geom;

            // ── Recreate geometry from FULL params ─────────────────────────────
            if (obj.type == PrimitiveType::SPHERE && obj.params.size() >= 1)
            {
                geom = std::make_shared<coal::Sphere>(obj.params[0]);
            }
            else if (obj.type == PrimitiveType::BOX && obj.params.size() >= 3)
            {
                geom = std::make_shared<coal::Box>(obj.params[0], obj.params[1], obj.params[2]);
            }
            else if (obj.type == PrimitiveType::CYLINDER && obj.params.size() >= 2)
            {
                geom = std::make_shared<coal::Cylinder>(obj.params[0], obj.params[1]);
            }
            else if (obj.type == PrimitiveType::CAPSULE && obj.params.size() >= 2)
            {
                geom = std::make_shared<coal::Capsule>(obj.params[0], obj.params[1]);
            }
            else if (obj.type == PrimitiveType::CONE && obj.params.size() >= 2)
            {
                geom = std::make_shared<coal::Cone>(obj.params[0], obj.params[1]);
            }
            else if (obj.type == PrimitiveType::MESH)
            {
                geom = obj.mesh_geom; // Use stored mesh
            }
            else
            {
                std::cout << "[Scene] ⚠️ Unknown or invalid object: " << obj.name << "\n";
                continue;
            }

            // ── Pose ─────────────────────────────────────────────────────────
            pinocchio::SE3 placement(obj.orientation.toRotationMatrix(), obj.position);

            pinocchio::GeometryObject new_obj(obj.name, 0, geom, placement);

            pinocchio::GeomIndex new_idx = geom_model_.addGeometryObject(new_obj);

            // ── Collision pairs: object vs robot ──────────────────────────────
            for (std::size_t i = 0; i < robot_geom_count_; ++i)
            {
                geom_model_.addCollisionPair(pinocchio::CollisionPair(i, new_idx));
            }

            // ── Collision pairs: object vs previous objects ───────────────────  // disable collision between scene objects
            // for (std::size_t i = robot_geom_count_; i < new_idx; ++i)
            // {
            //     geom_model_.addCollisionPair(pinocchio::CollisionPair(i, new_idx));
            // }
        }

        // ── Allow base ↔ ground (optional) ───────────────────────────────────
        pinocchio::GeomIndex ground_idx = geom_model_.geometryObjects.size();

        for (std::size_t i = robot_geom_count_; i < geom_model_.geometryObjects.size(); ++i)
        {
            if (geom_model_.geometryObjects[i].name == "ground_plane")
            {
                ground_idx = i;
                break;
            }
        }

        if (ground_idx < geom_model_.geometryObjects.size())
        {
            geom_model_.collisionPairs.erase(
                std::remove_if(geom_model_.collisionPairs.begin(), geom_model_.collisionPairs.end(),
                               [&](const pinocchio::CollisionPair &pair)
                               {

                                   bool involves_ground = (pair.first == ground_idx || pair.second == ground_idx);

                                   if (!involves_ground)
                                       return false; // Keep pairs that don't involve ground

                                   // Get the other object in the pair
                                   pinocchio::GeomIndex other_idx = (pair.first == ground_idx) ? pair.second : pair.first;

                                   // Remove if it's link0 (base)
                                   bool is_link0 = (geom_model_.geometryObjects[other_idx].name.find("link0") != std::string::npos);

                                   // Remove if it's a scene object (index >= robot_geom_count_)
                                   bool is_scene_object = (other_idx >= robot_geom_count_ && other_idx != ground_idx);

                                   return is_link0 || is_scene_object;
                               }),
                geom_model_.collisionPairs.end());
        }

        // ── Finalize ─────────────────────────────────────────────────────────
        geom_data_ = pinocchio::GeometryData(geom_model_);

        // Rebuild index list
        scene_object_indices_.clear();
        for (std::size_t i = robot_geom_count_; i < geom_model_.geometryObjects.size(); ++i)
        {
            scene_object_indices_.push_back(i);
        }

        std::cout << "[Scene] 🔁 Rebuilt geom model | total objects: "
                  << geom_model_.geometryObjects.size()
                  << " | pairs: " << geom_model_.collisionPairs.size() << "\n";
    }

    bool CheckCollision::removeSceneObject(const std::string &name)
    {
        std::lock_guard<std::mutex> lock(scene_mutex_);

        // Find in scene_objects_ list by name
        auto it = std::find_if(scene_objects_.begin(), scene_objects_.end(), [&name](const SceneObjectInfo &obj)
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
                info.type = PrimitiveType::SPHERE;
                info.params = {s->radius}; // [radius]
                break;
            }
            case coal::GEOM_CYLINDER:
            {
                const auto *c = dynamic_cast<const coal::Cylinder *>(geom.get());
                info.type = PrimitiveType::CYLINDER;
                info.params = {c->radius, c->halfLength * 2.0}; // [radius, full_length]
                break;
            }
            case coal::GEOM_BOX:
            {
                const auto *b = dynamic_cast<const coal::Box *>(geom.get());
                info.type = PrimitiveType::BOX;

                std::cout << "[DEBUG] Box halfSide: "
                          << b->halfSide[0] << ", "
                          << b->halfSide[1] << ", "
                          << b->halfSide[2] << std::endl;

                std::cout << "[DEBUG] Box full params: "
                          << b->halfSide[0] * 2.0 << ", "
                          << b->halfSide[1] * 2.0 << ", "
                          << b->halfSide[2] * 2.0 << std::endl;
                info.params = {b->halfSide[0] * 2.0,
                               b->halfSide[1] * 2.0,
                               b->halfSide[2] * 2.0}; // [lx, ly, lz]
                break;
            }
            case coal::GEOM_CAPSULE:
            {
                const auto *c = dynamic_cast<const coal::Capsule *>(geom.get());
                info.type = PrimitiveType::CAPSULE;
                info.params = {c->radius, c->halfLength * 2.0}; // [radius, full_shaft_length]
                break;
            }
            case coal::GEOM_CONE:
            {
                const auto *c = dynamic_cast<const coal::Cone *>(geom.get());
                info.type = PrimitiveType::CONE;
                info.params = {c->radius, c->halfLength * 2.0}; // [radius, full_height]
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
                    info.type = PrimitiveType::MESH;
                    info.params = {
                        static_cast<double>(aabb_max[0] - aabb_min[0]), // x extent
                        static_cast<double>(aabb_max[1] - aabb_min[1]), // y extent
                        static_cast<double>(aabb_max[2] - aabb_min[2])  // z extent
                    };
                }
                else
                {
                    info.type = PrimitiveType::MESH;
                    info.params = {};
                }
                break;
            }
            default:
                info.type = PrimitiveType::MESH;
                info.params = {};
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
                      << "\n"
                      << "  Dims : [";
            for (size_t i = 0; i < obj.params.size(); ++i)
                std::cout << obj.params[i] << (i + 1 < obj.params.size() ? ", " : "");
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