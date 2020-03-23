#include <memory>
#include <time.h>
#include <vector>
#include <numeric>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rttest/utils.h>
#include "common/tw_node_options.hpp"
#include "common/tw_utils.hpp"
#include "twmsgs/msg/data.hpp"
#include "setting.hpp"

class SubNode : public rclcpp::Node
{
public:
  explicit SubNode(std::string node_name,
      const rclcpp::NodeOptions & options,
      int index)
      : Node(node_name, options), index_(index), count_(0)
  {
    auto callback_sub =
        [this](const twmsgs::msg::Data::SharedPtr msg) -> void
        {
          auto time_sent = msg->time_sent_ns;
          struct timespec now;
          getnow(&now);

          std::cout << "sub" << index_
                    << " latency = " << timespec_to_long(&now) - time_sent
                    << " count = " << msg->data
                    << std::endl;
          // a long procedure
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        };
    std::cout << g_topic_name << std::endl;
    ping_sub_ =
        create_subscription<twmsgs::msg::Data>(g_topic_name,
                                               g_qos,
                                               callback_sub);
  }

private:
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr ping_sub_;
  int index_;
  int count_;
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

  std::vector<std::shared_ptr<rclcpp::Node> > sub_nodes;
  for(int i=0; i<max_subscription; i++) {
    std::string node_name = "sub_node" + std::to_string(i);
    auto sub_node = std::make_shared<SubNode>(node_name, node_options, i);
    exec->add_node(sub_node);
    sub_nodes.push_back(sub_node);
  }

  exec->spin();

  for(int i=0; i<max_subscription; i++) {
    exec->remove_node(sub_nodes[i]);
    sub_nodes[i] = nullptr;
  }

  rclcpp::shutdown();
}

