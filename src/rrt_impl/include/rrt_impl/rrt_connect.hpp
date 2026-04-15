#pragma once

#include "rrt_impl/rrt.hpp"

namespace rrt_core
{

    class RRTConnect : public RRT
    {
    public:
        explicit RRTConnect(size_t dof, const std::string &urdf_path);

        explicit RRTConnect(size_t dof, std::shared_ptr<collision::CheckCollision> collision);  


        // Main planner
        bool plan(int max_iter) override;

        std::vector<std::vector<double>> getPath() const override;

    private:
        // ================= INTERNAL TREE =================
        struct Tree
        {
            std::vector<RRTNode> nodes;
            KDTree kd_tree;

            Tree(size_t dof) : kd_tree(dof) {}
        };

        // ================= EXTEND STATUS =================
        enum class ExtendStatus
        {
            TRAPPED,
            ADVANCED,
            REACHED
        };

        // ================= CORE METHODS =================
        ExtendStatus extend(Tree &tree,
                            const std::vector<double> &q_target,
                            int &new_idx);

        ExtendStatus connect(Tree &tree,
                             const std::vector<double> &q_target,
                             int &new_idx);

        void buildFinalPath(Tree &Ta, int idxA,
                            Tree &Tb, int idxB);

        // ================= FINAL PATH =================
        std::vector<std::vector<double>> final_path_;
    };

} // namespace rrt_core