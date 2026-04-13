#include "rrt_impl/rrt_connect.hpp"
#include <algorithm>
#include <iostream>

namespace rrt_core
{

    // ================= CONSTRUCTOR =================
    RRTConnect::RRTConnect(size_t dof, const std::string &urdf_path)
        : RRT(dof, urdf_path)
    {
        std::cout << "RRT-Connect Constructed with DOF: " << dof_ << std::endl;

        // RRT-Connect works best without goal bias
        goal_bias_ = 0.0;
    }

    // ================= EXTEND =================
    RRTConnect::ExtendStatus RRTConnect::extend(Tree &tree,
                                                const std::vector<double> &q_target,
                                                int &new_idx)
    {
        int nearest_idx = tree.kd_tree.nearest(q_target);
        const auto &q_near = tree.nodes[nearest_idx].q;

        auto q_new = steer(q_near, q_target);

        if (!isEdgeCollisionFree(q_near, q_new))
            return ExtendStatus::TRAPPED;

        tree.nodes.push_back({q_new, nearest_idx});
        new_idx = tree.nodes.size() - 1;

        tree.kd_tree.insert(q_new, new_idx);

        if (distance(q_new, q_target) < goal_tolerance_)
            return ExtendStatus::REACHED;

        return ExtendStatus::ADVANCED;
    }

    // ================= CONNECT (GREEDY) =================
    RRTConnect::ExtendStatus RRTConnect::connect(Tree &tree,
                                                 const std::vector<double> &q_target,
                                                 int &new_idx)
    {
        ExtendStatus status;

        do
        {
            status = extend(tree, q_target, new_idx);
        } while (status == ExtendStatus::ADVANCED);

        return status;
    }

    // ================= MAIN PLANNER =================
    bool RRTConnect::plan(int max_iter)
    {
        std::cout << "[RRT-Connect] Planner parameters:" << std::endl;
        std::cout << "  DOF: " << dof_ << std::endl;
        std::cout << "  Step size: " << step_size_ << std::endl;
        std::cout << "  Goal tolerance: " << goal_tolerance_ << std::endl;
        std::cout << "  Max iterations: " << max_iter << std::endl;

        // Safety checks
        if (q_start_.empty() || q_goal_.empty())
        {
            throw std::runtime_error("Start or Goal not set");
        }

        if (lower_limits_.empty())
        {
            throw std::runtime_error("Joint limits not set");
        }

        final_path_.clear();

        // ================= INIT TREES =================
        Tree tree_start(dof_);
        Tree tree_goal(dof_);

        tree_start.nodes.push_back({q_start_, -1});
        tree_start.kd_tree.insert(q_start_, 0);

        tree_goal.nodes.push_back({q_goal_, -1});
        tree_goal.kd_tree.insert(q_goal_, 0);

        Tree *Ta = &tree_start;
        Tree *Tb = &tree_goal;

        // ================= MAIN LOOP =================
        for (int i = 0; i < max_iter; ++i)
        {
            auto q_rand = sampleRandomConfig();

            int idxA;
            auto status = extend(*Ta, q_rand, idxA);

            if (status != ExtendStatus::TRAPPED)
            {
                const auto &q_new = Ta->nodes[idxA].q;

                int idxB;
                auto status_connect = connect(*Tb, q_new, idxB);

                if (status_connect == ExtendStatus::REACHED)
                {
                    std::cout << "[RRT-Connect] Path found in " << i << " iterations\n";

                    buildFinalPath(*Ta, idxA, *Tb, idxB);
                    return true;
                }
            }

            // 🔁 Swap trees
            std::swap(Ta, Tb);
        }

        std::cout << "[RRT-Connect] Failed to find path\n";
        return false;
    }

    // ================= PATH MERGE =================
    void RRTConnect::buildFinalPath(Tree &Ta, int idxA,
                                   Tree &Tb, int idxB)
    {
        std::vector<std::vector<double>> pathA, pathB;

        // Backtrack Tree A
        while (idxA != -1)
        {
            pathA.push_back(Ta.nodes[idxA].q);
            idxA = Ta.nodes[idxA].parent_index;
        }

        // Backtrack Tree B
        while (idxB != -1)
        {
            pathB.push_back(Tb.nodes[idxB].q);
            idxB = Tb.nodes[idxB].parent_index;
        }

        std::reverse(pathA.begin(), pathA.end());

        // ⚠️ DO NOT reverse pathB
        pathA.insert(pathA.end(), pathB.begin(), pathB.end());

        final_path_ = pathA;
    }

    // ================= GET PATH =================
    std::vector<std::vector<double>> RRTConnect::getPath() const
    {
        return final_path_;
    }

} // namespace rrt_core