#pragma once
#include "rrt_impl/rrt.hpp"

namespace rrt_core
{
    class RRTStar : public RRT
    {
    public:
        RRTStar(size_t dof, const std::string &urdf_path);

        void setRewireRadius(double r);
        bool plan(int max_iter);

        double getCost(int idx) const;

    private:
        double rewire_radius_ = -1.0;
        std::vector<double> cost_;

        std::vector<int> getNeighbors(const std::vector<double> &q, double radius) const;
        double computeRadius() const;
        void propagateCostUpdate(int parent_idx, double parent_cost);
    };

} // namespace rrt_core