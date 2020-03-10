#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "../common/TwoWaysNode.hpp"
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
  rclcpp::executors::SingleThreadedExecutor exec;

  TwoWaysNodeOptions tw_options;
  rclcpp::NodeOptions node_options;
  // TODO: node_options.allocator = ...;
  // TODO: node_options.use_intra_process_comms = ...;
  auto n = std::make_shared<PubSubNode>(tw_options, node_options);

  exec.add_node(n);
  exec.spin();
  exec.remove_node(n);

  auto num_bin = tw_options.num_bin;
  print_result("wakeup_jitters", n->ping_wakeup_jitters_, n->ping_pub_count_, num_bin);
  print_result("recv_jitters",   n->ping_sub_jitters_,    n->ping_sub_count_, num_bin);

  rclcpp::shutdown();
}
