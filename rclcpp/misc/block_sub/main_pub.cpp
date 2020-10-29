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
  TwoWaysNodeOptions tw_options(argc, argv);

  if(lock_and_prefault_dynamic(tw_options.prefault_dynamic_size) < 0) {
    std::cerr << "lock_and_prefault_dynamic failed" << std::endl;
  }

  // setup child process scheule
  size_t priority = 0;
  int policy = 0;
  tw_options.get_child_thread_policy(priority, policy);
  set_sched_priority("child", priority, policy);

  rclcpp::init(argc, argv);
  auto exec = tw_options.get_executor();

  rclcpp::NodeOptions node_options;
  auto pub_node = std::make_shared<PubNode>(node_options);
  exec->add_node(pub_node);

  // setup child process scheule
  tw_options.get_main_thread_policy(priority, policy);
  set_sched_priority("main", priority, policy);

  exec->spin();

  exec->remove_node(pub_node);

  rclcpp::shutdown();
}
