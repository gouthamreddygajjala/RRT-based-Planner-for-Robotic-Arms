#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <ruckig/ruckig.hpp>

namespace rrt_core
{
    struct RuckigTrajectory
    {
        std::vector<std::vector<double>> positions;
        std::vector<std::vector<double>> velocities;
        std::vector<std::vector<double>> accelerations;
        double duration;
    };

    class RuckigSmoother
    {
    public:
        RuckigSmoother(size_t dof, double cycle_time = 0.01)
            : dof_(dof), cycle_time_(cycle_time) {}

        // Single segment — start to goal directly
        RuckigTrajectory smooth(
            const std::vector<double> &q_start,
            const std::vector<double> &q_goal,
            const std::vector<double> &max_vel,
            const std::vector<double> &max_acc,
            const std::vector<double> &max_jerk)
        {
            ruckig::Ruckig<ruckig::DynamicDOFs> ruckig(dof_, cycle_time_);
            ruckig::InputParameter<ruckig::DynamicDOFs> input(dof_);
            ruckig::OutputParameter<ruckig::DynamicDOFs> output(dof_);

            input.current_position     = q_start;
            input.current_velocity     = std::vector<double>(dof_, 0.0);
            input.current_acceleration = std::vector<double>(dof_, 0.0);

            input.target_position     = q_goal;
            input.target_velocity     = std::vector<double>(dof_, 0.0);
            input.target_acceleration = std::vector<double>(dof_, 0.0);

            input.max_velocity     = max_vel;
            input.max_acceleration = max_acc;
            input.max_jerk         = max_jerk;

            RuckigTrajectory result;

            ruckig::Result res;
            while ((res = ruckig.update(input, output)) == ruckig::Result::Working)
            {
                result.positions.push_back(output.new_position);
                result.velocities.push_back(output.new_velocity);
                result.accelerations.push_back(output.new_acceleration);
                output.pass_to_input(input);
            }

            if (res == ruckig::Result::Finished)
            {
                result.positions.push_back(output.new_position);
                result.velocities.push_back(output.new_velocity);
                result.accelerations.push_back(output.new_acceleration);
            }

            result.duration = result.positions.size() * cycle_time_;
            return result;
        }

        // MoveIt-style — positions fixed, ruckig only assigns vel/acc/time
        RuckigTrajectory timeParametrize(
            const std::vector<std::vector<double>> &path,
            const std::vector<double> &max_vel,
            const std::vector<double> &max_acc,
            const std::vector<double> &max_jerk)
        {
            if (path.size() < 2)
                throw std::runtime_error("Path must have at least 2 waypoints");

            // For 2 points — delegate to smooth directly
            if (path.size() == 2)
                return smooth(path[0], path[1], max_vel, max_acc, max_jerk);

            ruckig::Ruckig<ruckig::DynamicDOFs> ruckig_solver(dof_, cycle_time_);
            ruckig::InputParameter<ruckig::DynamicDOFs> input(dof_);
            ruckig::OutputParameter<ruckig::DynamicDOFs> output(dof_);

            // Start from rest
            input.current_position     = path.front();
            input.current_velocity     = std::vector<double>(dof_, 0.0);
            input.current_acceleration = std::vector<double>(dof_, 0.0);

            // Target is the goal
            input.target_position     = path.back();
            input.target_velocity     = std::vector<double>(dof_, 0.0);
            input.target_acceleration = std::vector<double>(dof_, 0.0);

            input.max_velocity     = max_vel;
            input.max_acceleration = max_acc;
            input.max_jerk         = max_jerk;

            RuckigTrajectory result;

            ruckig::Result res;
            while ((res = ruckig_solver.update(input, output)) == ruckig::Result::Working)
            {
                result.positions.push_back(output.new_position);
                result.velocities.push_back(output.new_velocity);
                result.accelerations.push_back(output.new_acceleration);
                output.pass_to_input(input);
            }

            if (res == ruckig::Result::Finished)
            {
                result.positions.push_back(output.new_position);
                result.velocities.push_back(output.new_velocity);
                result.accelerations.push_back(output.new_acceleration);
            }

            result.duration = result.positions.size() * cycle_time_;
            return result;
        }

        // Multi-waypoint — smooth through all waypoints segment by segment
        RuckigTrajectory smoothPath(
            const std::vector<std::vector<double>> &path,
            const std::vector<double> &max_vel,
            const std::vector<double> &max_acc,
            const std::vector<double> &max_jerk)
        {
            if (path.size() < 2)
                throw std::runtime_error("Path must have at least 2 waypoints");

            if (path.size() == 2)
                return smooth(path[0], path[1], max_vel, max_acc, max_jerk);

            RuckigTrajectory full;

            std::vector<double> current_vel(dof_, 0.0);
            std::vector<double> current_acc(dof_, 0.0);

            for (size_t i = 0; i < path.size() - 1; ++i)
            {
                ruckig::Ruckig<ruckig::DynamicDOFs> ruckig(dof_, cycle_time_);
                ruckig::InputParameter<ruckig::DynamicDOFs> input(dof_);
                ruckig::OutputParameter<ruckig::DynamicDOFs> output(dof_);

                input.current_position     = path[i];
                input.current_velocity     = current_vel;
                input.current_acceleration = current_acc;

                input.target_position     = path[i + 1];
                input.target_acceleration = std::vector<double>(dof_, 0.0);

                if (i == path.size() - 2)
                    input.target_velocity = std::vector<double>(dof_, 0.0);
                else
                    input.target_velocity = estimateVelocityFromGeometry(path, i + 1, max_vel);

                input.max_velocity     = max_vel;
                input.max_acceleration = max_acc;
                input.max_jerk         = max_jerk;

                ruckig::Result res;
                while ((res = ruckig.update(input, output)) == ruckig::Result::Working)
                {
                    full.positions.push_back(output.new_position);
                    full.velocities.push_back(output.new_velocity);
                    full.accelerations.push_back(output.new_acceleration);
                    output.pass_to_input(input);
                }

                if (res == ruckig::Result::Finished)
                {
                    full.positions.push_back(output.new_position);
                    full.velocities.push_back(output.new_velocity);
                    full.accelerations.push_back(output.new_acceleration);
                }

                current_vel = output.new_velocity;
                current_acc = output.new_acceleration;
            }

            full.duration = full.positions.size() * cycle_time_;
            return full;
        }

    private:
        size_t dof_;
        double cycle_time_;

        std::vector<double> estimateVelocityFromGeometry(
            const std::vector<std::vector<double>> &path,
            size_t i,
            const std::vector<double> &max_vel)
        {
            std::vector<double> vel(dof_, 0.0);

            if (i == 0 || i + 1 >= path.size()) return vel;

            double dt_prev = chordLength(path[i - 1], path[i]);
            double dt_next = chordLength(path[i],     path[i + 1]);
            double dt      = (dt_prev + dt_next) * 0.5;

            if (dt < 1e-6) return vel;

            for (size_t j = 0; j < dof_; ++j)
            {
                vel[j] = (path[i + 1][j] - path[i - 1][j]) / (2.0 * dt);
                vel[j] = std::clamp(vel[j], -max_vel[j], max_vel[j]);
            }

            return vel;
        }

        double chordLength(
            const std::vector<double> &a,
            const std::vector<double> &b)
        {
            double sum = 0.0;
            for (size_t j = 0; j < dof_; ++j)
            {
                double d = b[j] - a[j];
                sum += d * d;
            }
            return std::sqrt(sum);
        }
    };

} // namespace rrt_core