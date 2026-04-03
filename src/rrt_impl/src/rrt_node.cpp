#include "rclcpp/rclcpp.hpp"
#include "rrt_impl/rrt.hpp"
#include <sstream>

class RRTNode : public rclcpp::Node
{
public:
    RRTNode()
        : Node("rrt_node"), rrt_(7, "/home/gautam/Music/my_ws/rrt_planning/src/panda.urdf")
    {
        run();
    }

private:
    void run()
    {
        std::vector<double> q_start = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
        std::vector<double> q_goal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        rrt_.setStart(q_start);
        rrt_.setGoal(q_goal);

        std::vector<double> lower = {-2.9, -1.8, -2.9, -3.1, -2.9, -0.08, -2.9};
        std::vector<double> upper = {2.9, 1.8, 2.9, 0.08, 2.9, 3.8, 2.9};

        rrt_.setJointLimits(lower, upper);

        bool success = rrt_.plan(1000);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Path found!");

            auto path = rrt_.getPath();

            RCLCPP_INFO(this->get_logger(), "Path found! Length: %ld", path.size());

            for (size_t i = 0; i < path.size(); ++i)
            {
                std::ostringstream oss;
                oss << "Waypoint " << i << ": [ ";

                for (double val : path[i])
                {
                    oss << val << " ";
                }

                oss << "]";

                RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed!");
        }
    }

    rrt_core::RRT rrt_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}