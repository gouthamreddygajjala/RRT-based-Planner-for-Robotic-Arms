#pragma once
#include <vector>
#include <memory>
#include <limits>
#include <cmath>

namespace rrt_core
{
    struct KDNode
    {
        std::vector<double> q;
        int node_index;         // index into RRT's nodes_ vector
        std::unique_ptr<KDNode> left;
        std::unique_ptr<KDNode> right;
    };

    class KDTree
    {
    public:
        KDTree(size_t dof) : dof_(dof), root_(nullptr) {}

        void insert(const std::vector<double> &q, int node_index)
        {
            root_ = insert(std::move(root_), q, node_index, 0);
        }

        int nearest(const std::vector<double> &q) const
        {
            int best_index = -1;
            double best_dist = std::numeric_limits<double>::max();
            nearest(root_.get(), q, 0, best_dist, best_index);
            return best_index;
        }

        void clear()
        {
            root_.reset();
        }

    private:
        size_t dof_;
        std::unique_ptr<KDNode> root_;

        std::unique_ptr<KDNode> insert(std::unique_ptr<KDNode> node,
                                       const std::vector<double> &q,
                                       int node_index,
                                       int depth)
        {
            if (!node)
            {
                auto new_node = std::make_unique<KDNode>();
                new_node->q = q;
                new_node->node_index = node_index;
                return new_node;
            }

            int dim = depth % dof_;

            if (q[dim] < node->q[dim])
                node->left = insert(std::move(node->left), q, node_index, depth + 1);
            else
                node->right = insert(std::move(node->right), q, node_index, depth + 1);

            return node;
        }

        void nearest(const KDNode *node,
                     const std::vector<double> &q,
                     int depth,
                     double &best_dist,
                     int &best_index) const
        {
            if (!node) return;

            double dist = euclidean(node->q, q);

            if (dist < best_dist)
            {
                best_dist = dist;
                best_index = node->node_index;
            }

            int dim = depth % dof_;
            double diff = q[dim] - node->q[dim];

            // Search closer side first
            const KDNode *closer  = diff < 0 ? node->left.get()  : node->right.get();
            const KDNode *farther = diff < 0 ? node->right.get() : node->left.get();

            nearest(closer, q, depth + 1, best_dist, best_index);

            // Only visit farther side if it could contain a closer point
            if (diff * diff < best_dist)
                nearest(farther, q, depth + 1, best_dist, best_index);
        }

        double euclidean(const std::vector<double> &a, const std::vector<double> &b) const
        {
            double sum = 0.0;
            for (size_t i = 0; i < dof_; ++i)
            {
                double d = a[i] - b[i];
                sum += d * d;
            }
            return std::sqrt(sum);
        }
    };

} // namespace rrt_core