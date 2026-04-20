#include "rrt_impl/kinematics.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace kinematics
{

    Kinematics::Kinematics(const pinocchio::Model &model, pinocchio::FrameIndex ee_frame_id) : model_(model),
                                                                                               data_(model),
                                                                                               ee_frame_id_(ee_frame_id)
    {
        dof_ = model_.nq;

        q_min_ = model_.lowerPositionLimit;
        q_max_ = model_.upperPositionLimit;
    }

    -- -

       bool
       Kinematics::computeFK(const std::vector<double> &q, Eigen::Isometry3d &pose_out)
    {
        if ((int)q.size() != dof_)
            return false;

        Eigen::VectorXd qv = Eigen::Map<const Eigen::VectorXd>(q.data(), dof_);

        pinocchio::forwardKinematics(model_, data_, qv);
        pinocchio::updateFramePlacements(model_, data_);

        const auto &frame = data_.oMf[ee_frame_id_];

        pose_out = frame;
        return true;
    }

    -- -

       void
       Kinematics::computeJacobian(const std::vector<double> &q, Eigen::MatrixXd &J_out)
    {
        Eigen::VectorXd qv = Eigen::Map<const Eigen::VectorXd>(q.data(), dof_);

        pinocchio::computeFrameJacobian(
            model_, data_, qv,
            ee_frame_id_,
            pinocchio::LOCAL_WORLD_ALIGNED,
            J_out);
    }

    -- -

       bool
       Kinematics::computeIK(const geometry_msgs::msg::Pose &pose, std::vector<double> &q_out, const std::vector<double> &q_init)
    {
        Eigen::Vector3d target_pos(pose.position.x,
                                   pose.position.y,
                                   pose.position.z);

        Eigen::Quaterniond q_target(pose.orientation.w,
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z);
        q_target.normalize();

        Eigen::Matrix3d target_rot = q_target.toRotationMatrix();

        Eigen::VectorXd q;

        if (q_init.empty())
            q = Eigen::VectorXd::Zero(dof_);
        else
            q = Eigen::Map<const Eigen::VectorXd>(q_init.data(), dof_);

        for (int iter = 0; iter < max_iters_; ++iter)
        {
            pinocchio::forwardKinematics(model_, data_, q);
            pinocchio::updateFramePlacements(model_, data_);

            const auto &frame = data_.oMf[ee_frame_id_];

            Eigen::Vector3d curr_pos = frame.translation();
            Eigen::Matrix3d curr_rot = frame.rotation();

            Eigen::Vector3d pos_err = target_pos - curr_pos;

            Eigen::Matrix3d R_err = target_rot * curr_rot.transpose();
            Eigen::AngleAxisd aa(R_err);
            Eigen::Vector3d rot_err = aa.axis() * aa.angle();

            Eigen::Matrix<double, 6, 1> err;
            err.head<3>() = pos_err;
            err.tail<3>() = rot_err;

            if (err.norm() < ik_eps_)
            {
                q_out.assign(q.data(), q.data() + q.size());
                return true;
            }

            Eigen::MatrixXd J(6, dof_);
            pinocchio::computeFrameJacobian(
                model_, data_, q,
                ee_frame_id_,
                pinocchio::LOCAL_WORLD_ALIGNED,
                J);

            Eigen::MatrixXd JJt = J * J.transpose();
            Eigen::MatrixXd damping = damping_ * damping_ * Eigen::MatrixXd::Identity(6, 6);

            Eigen::VectorXd dq =
                J.transpose() * (JJt + damping).ldlt().solve(err);

            double alpha = 1.0;
            if (dq.norm() > max_step_)
                alpha = max_step_ / dq.norm();

            q += alpha * dq;

            for (int i = 0; i < dof_; ++i)
            {
                q[i] = std::clamp(q[i],
                                  q_min_[i],
                                  q_max_[i]);
            }
        }

        return false;
    }

} // namespace kinematics