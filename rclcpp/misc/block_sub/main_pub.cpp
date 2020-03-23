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

class PubNode : public rclcpp::Node
{
public:
  explicit PubNode(const rclcpp::NodeOptions & options)
      : Node("publisher", options), count_(0)
  {
    std::cout << g_topic_name << std::endl;
    ping_pub_ = this->create_publisher<twmsgs::msg::Data>(g_topic_name, g_qos);
    msg_ = std::make_shared<twmsgs::msg::Data>();
    auto publish_message =
        [this]() -> void
        {
          struct timespec time_wake_ts;
          getnow(&time_wake_ts);

          msg_->time_sent_ns = timespec_to_long(&time_wake_ts);
          msg_->data = count_;

          ping_pub_->publish(*msg_);
          count_++;
        };
    ping_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), publish_message);
  }

private:
  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr ping_pub_;
  std::shared_ptr<twmsgs::msg::Data> msg_;
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

  auto pub_node = std::make_shared<PubNode>(node_options);
  exec->add_node(pub_node);

  exec->spin();

  exec->remove_node(pub_node);

  rclcpp::shutdown();
}
