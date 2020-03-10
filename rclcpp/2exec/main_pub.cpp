#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"
#include "../common/PubNode.hpp"
#include "../common/tw_utils.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions node_options;
  // TODO: node_options.allocator = ...;
  // TODO: node_options.use_intra_process_comms = ...;
  auto npub = std::make_shared<PubNode>(node_options);

  exec.add_node(npub);
  exec.spin();
  exec.remove_node(npub);

  print_result("wakeup_jitters", npub->wakeup_jitters, npub->count, num_bin);

  rclcpp::shutdown();
}
