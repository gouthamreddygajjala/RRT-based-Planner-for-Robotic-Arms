#include "rrt_impl/rrt.hpp"

namespace rrt_core
{
    RRT::RRT(size_t dof, const std::string &urdf_path) : dof_(dof),
                                                         collision_checker_(urdf_path)
    {
        std::random_device rd;
        rng_ = std::mt19937(rd());
    }

    void RRT::setStart(const std::vector<double> &q)
    {
        validateConfig(q);

        q_start_ = q;

        nodes_.clear();
        nodes_.push_back({q_start_, -1}); // root node
    }

    void RRT::setGoal(const std::vector<double> &q)
    {
        validateConfig(q);
        q_goal_ = q;
    }

    void RRT::setJointLimits(const std::vector<double> &lower, const std::vector<double> &upper)
    {
        if (lower.size() != dof_ || upper.size() != dof_)
        {
            throw std::runtime_error("Joint limits size mismatch");
        }

        lower_limits_ = lower;
        upper_limits_ = upper;
    }

    void RRT::setStepSize(double step) { step_size_ = step; }
    void RRT::setGoalBias(double bias) { goal_bias_ = bias; }
    void RRT::setGoalTolerance(double tol) { goal_tolerance_ = tol; }
    void RRT::reserve(size_t n) { nodes_.reserve(n); }

    void RRT::validateConfig(const std::vector<double> &q) const
    {
        if (q.size() != dof_)
        {
            throw std::runtime_error("Configuration DOF mismatch");
        }

        // Optional (but useful): check joint limits if set
        if (!lower_limits_.empty() || !upper_limits_.empty())
        {
            for (size_t i = 0; i < dof_; ++i)
            {
                if (q[i] < lower_limits_[i] || q[i] > upper_limits_[i])
                {
                    throw std::runtime_error("Configuration violates joint limits");
                }
            }
        }
    }

    double RRT::distance(const std::vector<double> &a, const std::vector<double> &b) const // Joint Euclidean distance(not cartesian)
    {
        double sum = 0.0;

        for (size_t i = 0; i < dof_; ++i)
        {
            double d = a[i] - b[i];
            sum += d * d;
        }

        return std::sqrt(sum);
    }

    std::vector<double> RRT::sampleRandomConfig()
    {
        // Probability distribution for goal bias
        std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

        // Goal biasing
        if (prob_dist(rng_) < goal_bias_)
        {
            return q_goal_;
        }

        std::vector<double> q(dof_);

        for (size_t i = 0; i < dof_; ++i)
        {
            std::uniform_real_distribution<double> dist(
                lower_limits_[i],
                upper_limits_[i]);

            q[i] = dist(rng_);
        }

        return q;
    }

    int RRT::getNearestNode(const std::vector<double> &q)
    {
        double min_dist = std::numeric_limits<double>::max();
        int nearest_idx = 0;

        for (size_t i = 0; i < nodes_.size(); ++i)
        {
            double d = distance(nodes_[i].q, q);

            if (d < min_dist)
            {
                min_dist = d;
                nearest_idx = static_cast<int>(i);
            }
        }

        return nearest_idx;
    }

    std::vector<double> RRT::steer(const std::vector<double> &q_near, const std::vector<double> &q_rand)
    {
        std::vector<double> q_new(dof_);

        double dist = distance(q_near, q_rand);

        // If already close, just return q_rand
        if (dist < step_size_)
        {
            return q_rand;
        }

        // Move from q_near toward q_rand
        for (size_t i = 0; i < dof_; ++i)
        {
            q_new[i] = q_near[i] +
                       step_size_ * (q_rand[i] - q_near[i]) / dist;
        }

        return q_new;
    }

    bool RRT::isEdgeCollisionFree(const std::vector<double> &q1,
                                  const std::vector<double> &q2) const
    {
        double dist = distance(q1, q2);

        // Resolution of interpolation (tunable)
        const double resolution = 0.05;

        int steps = static_cast<int>(dist / resolution);

        // Ensure at least one check
        steps = std::max(steps, 1);

        for (int i = 0; i <= steps; ++i)
        {
            double t = static_cast<double>(i) / steps;

            std::vector<double> q_interp(dof_);

            for (size_t j = 0; j < dof_; ++j)
            {
                q_interp[j] = q1[j] + t * (q2[j] - q1[j]);
            }

            if (!isCollisionFree(q_interp))
            {
                return false;
            }
        }

        return true;
    }

    bool RRT::plan(int max_iter)
    {
        // Safety checks
        if (q_start_.empty() || q_goal_.empty())
        {
            throw std::runtime_error("Start or Goal not set");
        }

        if (lower_limits_.empty())
        {
            throw std::runtime_error("Joint limits not set");
        }

        goal_index_ = -1;

        for (int i = 0; i < max_iter; ++i)
        {
            // 1. Sample
            auto q_rand = sampleRandomConfig();

            // 2. Nearest
            int nearest_idx = getNearestNode(q_rand);
            const auto &q_near = nodes_[nearest_idx].q;

            // 3. Steer
            auto q_new = steer(q_near, q_rand);

            // 4. Collision (edge check)
            if (!isEdgeCollisionFree(q_near, q_new))
                continue;

            // 5. Add node
            nodes_.push_back({q_new, nearest_idx});
            int new_idx = nodes_.size() - 1;

            // 6. Goal check
            if (distance(q_new, q_goal_) < goal_tolerance_)
            {
                // Connect goal
                nodes_.push_back({q_goal_, new_idx});
                goal_index_ = nodes_.size() - 1;

                return true; // success
            }
        }

        return false; // failed
    }

    std::vector<std::vector<double>> RRT::getPath() const
    {
        std::vector<std::vector<double>> path;

        // If planning failed
        if (goal_index_ == -1)
        {
            return path;
        }

        int idx = goal_index_;

        // Backtrack from goal to start
        while (idx != -1)
        {
            path.push_back(nodes_[idx].q);
            idx = nodes_[idx].parent_index;
        }

        // Reverse to get start → goal
        std::reverse(path.begin(), path.end());

        return path;
    }

    bool RRT::isCollisionFree(const std::vector<double> &q) const
    {
        if (collision_checker_.isStateValid(q))
        {
            std::cout << "State is valid" << std::endl;
        }
        else
        {
            std::cout << "State is in collision" << std::endl;
        }
        return collision_checker_.isStateValid(q);
    }

} // namespace rrt_core