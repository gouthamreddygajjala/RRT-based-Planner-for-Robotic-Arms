#pragma once

#include <vector>
#include <string>
#include <iostream>

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

namespace collision
{
    class CheckCollision
    {
    public:
        explicit CheckCollision(const std::string &urdf_path);
        bool isStateValid(const std::vector<double> &q) const;

    private:
        pinocchio::Model model_;
        mutable pinocchio::Data data_;

        pinocchio::GeometryModel geom_model_;
        mutable pinocchio::GeometryData geom_data_;

        int dof_;
    };
}