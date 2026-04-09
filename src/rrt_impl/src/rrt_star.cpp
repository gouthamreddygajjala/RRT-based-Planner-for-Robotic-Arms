#include "rrt_impl/rrt_star.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>
#include <queue>

namespace rrt_core
{

    RRTStar::RRTStar(size_t dof, const std::string &urdf_path) : RRT(dof, urdf_path)
    {
        std::cout << "RRT* Constructed with DOF: " << dof_ << std::endl;
    }

    void RRTStar::setRewireRadius(double r)
    {
        rewire_radius_ = r;
    }

    double RRTStar::getCost(int idx) const
    {
        return (idx >= 0 && idx < (int)cost_.size()) ? cost_[idx] : 0.0;
    }

    // Radius computation with clamp
    double RRTStar::computeRadius() const
    {
        size_t n = nodes_.size();

        if (n <= 1)
            return step_size_ * 2.0;

        const double gamma = 2.0 * std::pow(1.5 * (1.0 + 1.0 / dof_), 1.0 / dof_);
        double r = gamma * std::pow(std::log((double)n) / (double)n, 1.0 / dof_);

        return std::max(r, step_size_ * 2.0);
    }

    // O(n) neighbor search (replace later with KD-tree radius)
    // std::vector<int> RRTStar::getNeighbors(const std::vector<double> &q, double radius) const
    // {
    //     std::vector<int> neighbors;

    //     for (size_t i = 0; i < nodes_.size(); ++i)
    //     {
    //         if (distance(nodes_[i].q, q) <= radius)
    //             neighbors.push_back((int)i);
    //     }

    //     return neighbors;
    // }

    std::vector<int> RRTStar::getNeighbors(const std::vector<double> &q, double radius) const
    {
        auto neighbors = kd_tree_.radiusSearch(q, radius);

        // Performance cap (important)
        if (neighbors.size() > 50)
            neighbors.resize(50);

        return neighbors;
    }

    // BFS cost propagation
    void RRTStar::propagateCostUpdate(int start_idx)
    {
        std::queue<int> q;
        q.push(start_idx);

        while (!q.empty())
        {
            int parent = q.front();
            q.pop();

            for (size_t i = 0; i < nodes_.size(); ++i)
            {
                if (nodes_[i].parent_index == parent)
                {
                    cost_[i] = cost_[parent] +
                               distance(nodes_[parent].q, nodes_[i].q);

                    q.push((int)i);
                }
            }
        }
    }

    bool RRTStar::plan(int max_iter)
    {
        std::cout << "[RRT*] Planner parameters:" << std::endl;
        std::cout << "  DOF: " << dof_ << std::endl;
        std::cout << "  Step size: " << step_size_ << std::endl;
        std::cout << "  Goal bias: " << goal_bias_ << std::endl;
        std::cout << "  Goal tolerance: " << goal_tolerance_ << std::endl;
        std::cout << "  Max iterations: " << max_iter << std::endl;

        if (q_start_.empty() || q_goal_.empty())
            throw std::runtime_error("Start or Goal not set");

        if (lower_limits_.empty())
            throw std::runtime_error("Joint limits not set");

        cost_.clear();
        cost_.push_back(0.0);

        goal_index_ = -1;
        double best_goal_cost = std::numeric_limits<double>::max();

        int no_improve_counter = 0;
        const int max_no_improve = 200;

        for (int iter = 0; iter < max_iter; ++iter)
        {
            // Debug
            if (iter % 500 == 0)
            {
                std::cout << "[RRT*] Iter: " << iter
                          << " Nodes: " << nodes_.size()
                          << " Best cost: "
                          << (goal_index_ != -1 ? best_goal_cost : -1)
                          << std::endl;
            }

            bool improved_this_iter = false;

            // 1. Sample
            auto q_rand = sampleRandomConfig();

            // 2. Nearest
            int nearest_idx = getNearestNode(q_rand);
            const auto &q_near = nodes_[nearest_idx].q;

            // 3. Steer
            auto q_new = steer(q_near, q_rand);

            // 4. Collision checks
            if (!isCollisionFree(q_new))
                continue;

            if (!isEdgeCollisionFree(q_near, q_new))
                continue;

            // 5. Neighbor search
            double radius = (rewire_radius_ > 0.0) ? rewire_radius_ : computeRadius();
            auto neighbors = getNeighbors(q_new, radius);

            if (neighbors.size() > 50)
                neighbors.resize(50);

            // 6. Best parent
            int best_parent = nearest_idx;
            double best_cost = cost_[nearest_idx] +
                               distance(nodes_[nearest_idx].q, q_new);

            for (int nb_idx : neighbors)
            {
                if (!isEdgeCollisionFree(nodes_[nb_idx].q, q_new))
                    continue;

                double candidate_cost = cost_[nb_idx] +
                                        distance(nodes_[nb_idx].q, q_new);

                if (candidate_cost < best_cost)
                {
                    best_cost = candidate_cost;
                    best_parent = nb_idx;
                }
            }

            // 7. Add node
            nodes_.push_back({q_new, best_parent});
            cost_.push_back(best_cost);
            int new_idx = (int)nodes_.size() - 1;

            kd_tree_.insert(q_new, new_idx);

            // 8. Rewire
            for (int nb_idx : neighbors)
            {
                if (nb_idx == best_parent)
                    continue;

                if (!isEdgeCollisionFree(q_new, nodes_[nb_idx].q))
                    continue;

                double new_cost = best_cost +
                                  distance(q_new, nodes_[nb_idx].q);

                if (new_cost < cost_[nb_idx])
                {
                    nodes_[nb_idx].parent_index = new_idx;
                    cost_[nb_idx] = new_cost;

                    propagateCostUpdate(nb_idx);
                }
            }

            // 9. Goal check
            double dist_to_goal = distance(q_new, q_goal_);

            if (dist_to_goal < goal_tolerance_)
            {
                if (isEdgeCollisionFree(q_new, q_goal_))
                {
                    double goal_cost = best_cost + dist_to_goal;

                    if (goal_cost < best_goal_cost)
                    {
                        best_goal_cost = goal_cost;
                        improved_this_iter = true;

                        if (goal_index_ == -1)
                        {
                            nodes_.push_back({q_goal_, new_idx});
                            cost_.push_back(goal_cost);
                            goal_index_ = (int)nodes_.size() - 1;
                        }
                        else
                        {
                            nodes_[goal_index_].parent_index = new_idx;
                            cost_[goal_index_] = goal_cost;
                        }
                    }
                }
            }

            // ⭐ Correct early stopping logic
            if (goal_index_ != -1)
            {
                if (improved_this_iter)
                    no_improve_counter = 0;
                else
                    no_improve_counter++;
            }

            if (goal_index_ != -1 && no_improve_counter > max_no_improve)
            {
                std::cout << "[RRT*] Early stopping triggered after "
                          << no_improve_counter
                          << " iterations without improvement\n";
                break;
            }
        }

        std::cout << "[RRT*] Tree size: " << nodes_.size()
                  << "  Best goal cost: "
                  << (goal_index_ != -1 ? best_goal_cost : -1.0)
                  << std::endl;

        return goal_index_ != -1;
    }

} // namespace rrt_core