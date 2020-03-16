#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "../common/two_ways_node.hpp"
#include "../common/tw_utils.hpp"

const char * node_name = "one_node_pub_sub";

class PubSubNode : public TwoWaysNode
{
public:
  PubSubNode(
      const TwoWaysNodeOptions & tw_options,
      const rclcpp::NodeOptions & options)
      : TwoWaysNode(node_name, tw_options.namespace_, tw_options, options)
  {
    this->setup_ping_publisher();
    this->setup_ping_subscriber();
  }
};

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  rclcpp::init(argc, argv);
  TwoWaysNodeOptions tw_options;
  if (!tw_options.set_realtime_settings()) {
    std::cerr << "set_realtime_setting failed" << std::endl;;
    return -1;
  }

  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto n = std::make_shared<PubSubNode>(tw_options, node_options);

  exec->add_node(n);
  exec->spin();
  exec->remove_node(n);

  n->print_ping_wakeup_report();
  n->print_ping_sub_report();

  rclcpp::shutdown();
}
