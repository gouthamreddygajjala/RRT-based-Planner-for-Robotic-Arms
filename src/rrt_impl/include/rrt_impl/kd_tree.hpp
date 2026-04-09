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
        int node_index;
        std::unique_ptr<KDNode> left;
        std::unique_ptr<KDNode> right;
    };

    class KDTree
    {
    public:
        KDTree(size_t dof) : dof_(dof), root_(nullptr) {}

        // ================= INSERT =================
        void insert(const std::vector<double> &q, int node_index)
        {
            root_ = insert(std::move(root_), q, node_index, 0);
        }

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

        // ================= NEAREST =================
        int nearest(const std::vector<double> &q) const
        {
            int best_index = -1;
            double best_dist = std::numeric_limits<double>::max();
            nearest(root_.get(), q, 0, best_dist, best_index);
            return best_index;
        }

        // ================= RADIUS SEARCH (NEW) =================
        std::vector<int> radiusSearch(const std::vector<double> &q, double radius) const
        {
            std::vector<int> result;
            double radius_sq = radius * radius;

            radiusSearch(root_.get(), q, radius_sq, 0, result);

            return result;
        }

        void clear()
        {
            root_.reset();
        }

    private:
        size_t dof_;
        std::unique_ptr<KDNode> root_;

        // ================= NEAREST HELPER =================
        void nearest(const KDNode *node,
                     const std::vector<double> &q,
                     int depth,
                     double &best_dist,
                     int &best_index) const
        {
            if (!node)
                return;

            double dist = euclidean(node->q, q);

            if (dist < best_dist)
            {
                best_dist = dist;
                best_index = node->node_index;
            }

            int dim = depth % dof_;
            double diff = q[dim] - node->q[dim];

            const KDNode *near = diff < 0 ? node->left.get() : node->right.get();
            const KDNode *far = diff < 0 ? node->right.get() : node->left.get();

            nearest(near, q, depth + 1, best_dist, best_index);

            if (diff * diff < best_dist)
                nearest(far, q, depth + 1, best_dist, best_index);
        }

        // ================= RADIUS HELPER =================
        void radiusSearch(const KDNode *node,
                          const std::vector<double> &q,
                          double radius_sq,
                          int depth,
                          std::vector<int> &result) const
        {
            if (!node)
                return;

            // Check current node
            double dist_sq = squaredDistance(node->q, q);
            if (dist_sq <= radius_sq)
            {
                result.push_back(node->node_index);
            }

            int dim = depth % dof_;
            double diff = q[dim] - node->q[dim];

            const KDNode *near = diff < 0 ? node->left.get() : node->right.get();
            const KDNode *far = diff < 0 ? node->right.get() : node->left.get();

            // Always search near branch
            radiusSearch(near, q, radius_sq, depth + 1, result);

            // Search far branch only if needed
            if (diff * diff <= radius_sq)
            {
                radiusSearch(far, q, radius_sq, depth + 1, result);
            }
        }

        // ================= DISTANCE =================
        double euclidean(const std::vector<double> &a,
                         const std::vector<double> &b) const
        {
            return std::sqrt(squaredDistance(a, b));
        }

        double squaredDistance(const std::vector<double> &a,
                               const std::vector<double> &b) const
        {
            double sum = 0.0;
            for (size_t i = 0; i < dof_; ++i)
            {
                double d = a[i] - b[i];
                sum += d * d;
            }
            return sum;
        }
    };

} // namespace rrt_core