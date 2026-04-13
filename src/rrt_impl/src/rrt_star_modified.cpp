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

    double RRTStar::computeRadius() const
    {
        size_t n = nodes_.size();
        if (n <= 1)
            return step_size_ * 2.0;

        const double gamma = 2.0 * std::pow(1.5 * (1.0 + 1.0 / dof_), 1.0 / dof_);
        double r = gamma * std::pow(std::log((double)n) / (double)n, 1.0 / dof_);
        return std::max(r, step_size_ * 2.0);
    }

    std::vector<int> RRTStar::getNeighbors(const std::vector<double> &q, double radius) const
    {
        auto neighbors = kd_tree_.radiusSearch(q, radius);
        if (neighbors.size() > 50)
            neighbors.resize(50);
        return neighbors;
    }

    // O(subtree) — iterates children_ list instead of scanning all nodes
    void RRTStar::propagateCostUpdate(int start_idx)
    {
        std::queue<int> q;
        q.push(start_idx);

        while (!q.empty())
        {
            int parent = q.front();
            q.pop();

            for (int child : nodes_[parent].children)
            {
                cost_[child] = cost_[parent] +
                               distance(nodes_[parent].q, nodes_[child].q);
                q.push(child);
            }
        }
    }

    bool RRTStar::plan(int max_iter)
    {
        std::cout << "[RRT*] Planner parameters:" << std::endl;
        std::cout << "  DOF: "            << dof_            << std::endl;
        std::cout << "  Step size: "      << step_size_      << std::endl;
        std::cout << "  Goal bias: "      << goal_bias_      << std::endl;
        std::cout << "  Goal tolerance: " << goal_tolerance_ << std::endl;
        std::cout << "  Max iterations: " << max_iter        << std::endl;

        if (q_start_.empty() || q_goal_.empty())
            throw std::runtime_error("Start or Goal not set");

        if (lower_limits_.empty())
            throw std::runtime_error("Joint limits not set");

        // Root node — must already exist in nodes_[0] set by base class.
        // Ensure cost and children vectors are sized to match.
        cost_.clear();
        cost_.push_back(0.0);

        // If base class pushed the root node without children, initialise here.
        // Safe to call resize — if nodes_[0].children already exists this is a no-op.
        if (nodes_.size() == 1)
            nodes_[0].children.clear();

        goal_index_ = -1;
        double best_goal_cost = std::numeric_limits<double>::max();

        // Scale patience with DOF — high-DOF spaces improve less frequently
        const int max_no_improve = std::max(200, (int)(50 * dof_));
        int no_improve_counter = 0;

        for (int iter = 0; iter < max_iter; ++iter)
        {
            if (iter % 500 == 0)
            {
                std::cout << "[RRT*] Iter: " << iter
                          << " Nodes: " << nodes_.size()
                          << " Best cost: "
                          << (goal_index_ != -1 ? best_goal_cost : -1.0)
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

            // 5. Neighbor search (cap applied once, inside getNeighbors)
            double radius = (rewire_radius_ > 0.0) ? rewire_radius_ : computeRadius();
            auto neighbors = getNeighbors(q_new, radius);

            // 6. Best parent selection
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
                    best_cost   = candidate_cost;
                    best_parent = nb_idx;
                }
            }

            // 7. Add new node — empty children list, register with parent
            nodes_.push_back({q_new, best_parent, {}});
            cost_.push_back(best_cost);
            int new_idx = (int)nodes_.size() - 1;

            nodes_[best_parent].children.push_back(new_idx);
            kd_tree_.insert(q_new, new_idx);

            // 8. Rewire neighbors through the new node if cheaper
            for (int nb_idx : neighbors)
            {
                if (nb_idx == best_parent)
                    continue;

                if (!isEdgeCollisionFree(q_new, nodes_[nb_idx].q))
                    continue;

                double new_cost = best_cost + distance(q_new, nodes_[nb_idx].q);

                if (new_cost < cost_[nb_idx])
                {
                    // Remove nb_idx from its current parent's children list
                    int old_parent = nodes_[nb_idx].parent_index;
                    auto &old_ch   = nodes_[old_parent].children;
                    old_ch.erase(std::remove(old_ch.begin(), old_ch.end(), nb_idx),
                                 old_ch.end());

                    // Reparent
                    nodes_[nb_idx].parent_index = new_idx;
                    nodes_[new_idx].children.push_back(nb_idx);
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
                        best_goal_cost    = goal_cost;
                        improved_this_iter = true;

                        if (goal_index_ == -1)
                        {
                            // First time reaching goal — add goal node properly
                            nodes_.push_back({q_goal_, new_idx, {}});
                            cost_.push_back(goal_cost);
                            goal_index_ = (int)nodes_.size() - 1;

                            nodes_[new_idx].children.push_back(goal_index_);
                            kd_tree_.insert(q_goal_, goal_index_);
                        }
                        else
                        {
                            // Better path found — rewire goal node
                            int old_parent = nodes_[goal_index_].parent_index;
                            auto &old_ch   = nodes_[old_parent].children;
                            old_ch.erase(std::remove(old_ch.begin(), old_ch.end(), goal_index_),
                                         old_ch.end());

                            nodes_[goal_index_].parent_index = new_idx;
                            nodes_[new_idx].children.push_back(goal_index_);
                            cost_[goal_index_] = goal_cost;
                        }
                    }
                }
            }

            // Early stopping
            if (goal_index_ != -1)
            {
                if (improved_this_iter)
                    no_improve_counter = 0;
                else
                    no_improve_counter++;
            }

            if (goal_index_ != -1 && no_improve_counter > max_no_improve)
            {
                std::cout << "[RRT*] Early stopping after "
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