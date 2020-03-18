#include "cyclic_component.hpp"

using namespace cyclic_component;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    rclcpp::executor::Executor::SharedPtr exec
        = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::Node::SharedPtr node = std::make_shared<CyclicNode>(node_options);

    exec->add_node(node);
    exec->spin();
    exec->remove_node(node);

    rclcpp::shutdown();
}
