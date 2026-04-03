#include "rrt_impl/check_collision.hpp"

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

            geom_data_ = pinocchio::GeometryData(geom_model_);

            dof_ = model_.nq;

            std::cout << "URDF + Collision loaded. DOF: " << dof_ << std::endl;
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
        return false;

    Eigen::VectorXd q_eigen = Eigen::Map<const Eigen::VectorXd>(q.data(), dof_);

    // 6-arg overload: handles forwardKinematics + updateGeometryPlacements internally
    bool in_collision = pinocchio::computeCollisions(
        model_, data_, geom_model_, geom_data_, q_eigen, /*stopAtFirstCollision=*/true);

    return !in_collision;
}

} // namespace collision