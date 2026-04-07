#include "rrt_impl/rrt_star.hpp"
#include <cmath>
#include <limits>
#include <iostream>

namespace rrt_core
{
    RRTStar::RRTStar(size_t dof, const std::string &urdf_path) : RRT(dof, urdf_path)
    {
    }

    void RRTStar::setRewireRadius(double r) { rewire_radius_ = r; }

    double RRTStar::getCost(int idx) const
    {
        return (idx >= 0 && idx < (int)cost_.size()) ? cost_[idx] : 0.0;
    }

    double RRTStar::computeRadius() const
    {
        // Asymptotically optimal shrinking-ball radius:
        // r = gamma * (log(n) / n)^(1/d)
        // gamma > 2 * (1 + 1/d)^(1/d) * (vol(free) / vol(unit_ball))^(1/d)
        const double gamma = 2.0 * std::pow(1.5 * (1.0 + 1.0 / dof_), 1.0 / dof_);
        size_t n = nodes_.size();
        if (n <= 1)
            return step_size_ * 2.0;
        return gamma * std::pow(std::log((double)n) / (double)n, 1.0 / dof_);
    }

    std::vector<int> RRTStar::getNeighbors(const std::vector<double> &q, double radius) const
    {
        std::vector<int> neighbors;
        for (size_t i = 0; i < nodes_.size(); ++i)
        {
            if (distance(nodes_[i].q, q) <= radius)
                neighbors.push_back((int)i);
        }
        return neighbors;
    }

    void RRTStar::propagateCostUpdate(int parent_idx, double parent_cost)
    {
        for (size_t i = 0; i < nodes_.size(); ++i)
        {
            if (nodes_[i].parent_index == parent_idx)
            {
                double edge_len = distance(nodes_[parent_idx].q, nodes_[i].q);
                cost_[i] = parent_cost + edge_len;
                propagateCostUpdate((int)i, cost_[i]);
            }
        }
    }

    bool RRTStar::plan(int max_iter)
    {
        // Safety checks
        if (q_start_.empty() || q_goal_.empty())
            throw std::runtime_error("Start or Goal not set");

        if (lower_limits_.empty())
            throw std::runtime_error("Joint limits not set");

        // Re-initialise tree (setStart already cleared nodes_)
        cost_.clear();
        cost_.push_back(0.0); // root node has zero cost
        goal_index_ = -1;
        double best_goal_cost = std::numeric_limits<double>::max();

        for (int iter = 0; iter < max_iter; ++iter)
        {
            // ── 1. Sample ──────────────────────────────────────────────────
            auto q_rand = sampleRandomConfig();

            // ── 2. Nearest ─────────────────────────────────────────────────
            int nearest_idx = getNearestNode(q_rand);
            const auto &q_near = nodes_[nearest_idx].q;

            // ── 3. Steer ───────────────────────────────────────────────────
            auto q_new = steer(q_near, q_rand);

            // ── 4. Edge collision check (nearest → new) ────────────────────
            if (!isEdgeCollisionFree(q_near, q_new))
                continue;

            // ── 5. Find neighbors within rewire radius ──────────────────────
            double radius = (rewire_radius_ > 0.0) ? rewire_radius_ : computeRadius();
            auto neighbors = getNeighbors(q_new, radius);

            // ── 6. Choose best parent from neighbors ───────────────────────
            int best_parent = nearest_idx;
            double best_cost = cost_[nearest_idx] + distance(nodes_[nearest_idx].q, q_new);

            for (int nb_idx : neighbors)
            {
                double candidate_cost = cost_[nb_idx] + distance(nodes_[nb_idx].q, q_new);
                if (candidate_cost < best_cost && isEdgeCollisionFree(nodes_[nb_idx].q, q_new))
                {
                    best_cost = candidate_cost;
                    best_parent = nb_idx;
                }
            }

            // ── 7. Add node ────────────────────────────────────────────────
            nodes_.push_back({q_new, best_parent});
            cost_.push_back(best_cost);
            int new_idx = (int)nodes_.size() - 1;

            // ── 8. Rewire neighbors through the new node ───────────────────
            for (int nb_idx : neighbors)
            {
                double rewired_cost = best_cost + distance(q_new, nodes_[nb_idx].q);
                if (rewired_cost < cost_[nb_idx] && isEdgeCollisionFree(q_new, nodes_[nb_idx].q))
                {
                    nodes_[nb_idx].parent_index = new_idx;
                    cost_[nb_idx] = rewired_cost;

                    // Propagate updated cost down the affected subtree
                    propagateCostUpdate(nb_idx, rewired_cost);
                }
            }

            // ── 9. Goal check – keep improving if a better path is found ───
            double dist_to_goal = distance(q_new, q_goal_);
            if (dist_to_goal < goal_tolerance_)
            {
                double goal_cost = best_cost + dist_to_goal;
                if (goal_cost < best_goal_cost)
                {
                    best_goal_cost = goal_cost;
                    if (goal_index_ == -1)
                    {
                        // First time we reach the goal – add goal node
                        nodes_.push_back({q_goal_, new_idx});
                        cost_.push_back(goal_cost);
                        goal_index_ = (int)nodes_.size() - 1;
                    }
                    else
                    {
                        // Better path found – update existing goal node
                        nodes_[goal_index_].parent_index = new_idx;
                        cost_[goal_index_] = goal_cost;
                    }
                }
            }
        }

        std::cout << "[RRT*] Tree size: " << nodes_.size()
                  << "  Best goal cost: "
                  << (goal_index_ != -1 ? best_goal_cost : -1.0)
                  << std::endl;

        return goal_index_ != -1;
    }

} // namespace rrt_core