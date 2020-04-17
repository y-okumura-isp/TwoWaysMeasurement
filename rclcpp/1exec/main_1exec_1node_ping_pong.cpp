#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "../common/two_ways_node.hpp"
#include "../common/tw_utils.hpp"

const char * node_name = "one_node_ping_pong";

class PubSubNode : public TwoWaysNode
{
public:
  PubSubNode(
      const TwoWaysNodeOptions & tw_options,
      const rclcpp::NodeOptions & options)
      : TwoWaysNode(node_name, tw_options.namespace_, tw_options, options)
  {
    this->setup_ping_publisher();
    this->setup_ping_subscriber(true);
    this->setup_pong_subscriber();
  }
};

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  TwoWaysNodeOptions tw_options(argc, argv);
  SET_REALTIME_SETTING_RRRR(tw_options);

  rclcpp::init(argc, argv);
  auto exec = tw_options.get_executor();
  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);
  auto n = std::make_shared<PubSubNode>(tw_options, node_options);

  exec->add_node(n);
  SET_REALTIME_SETTING_RRTS(tw_options);
  exec->spin();
  exec->remove_node(n);

  n->print_ping_wakeup_report();
  n->print_diff_wakeup_report();
  n->print_ping_sub_report();
  n->print_pong_sub_report();
  n->print_ping_pong_report();

  rclcpp::shutdown();
}
