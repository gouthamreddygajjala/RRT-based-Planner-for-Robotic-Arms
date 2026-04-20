#pragma once

#include <vector>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>

class Kinematics
{
public:
    Kinematics(const pinocchio::Model &model, pinocchio::FrameIndex ee_frame_id);

    bool computeIK(const geometry_msgs::msg::Pose &pose, std::vector<double> &q_out);

private:
    const pinocchio::Model &model_; // reference, not copy
    pinocchio::Data data_;          // own data

    pinocchio::FrameIndex ee_frame_id_;
    int dof_;

    Eigen::VectorXd q_min_;
    Eigen::VectorXd q_max_;
};