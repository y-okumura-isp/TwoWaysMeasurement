#include <rclcpp/rclcpp.hpp>

#include "../common/tw_node_options.hpp"
#include "../common/tw_utils.hpp"
#include "../common/two_ways_service_node.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;
  rclcpp::init(argc, argv);

  TwoWaysNodeOptions tw_options;
  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto n = std::make_shared<TwoWaysServiceNode>("hogenode", "");
  if (!n->setup_ping_client()) {
    std::cerr << "cannot setup client" << std::endl;
  }

  exec->add_node(n);
  exec->spin();
  exec->remove_node(n);
}
