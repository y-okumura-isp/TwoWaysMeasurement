#ifndef SUBNODE_HPP_
#define SUBNODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"

class SubNode : public rclcpp::Node
{
  using _SC = std::chrono::steady_clock;

public:
  SubNode(const rclcpp::NodeOptions & options)
      : Node(node_name_sub, options), count(0)
  {
    // sub
    auto callback_sub =
        [this](const twmsgs::msg::Data::SharedPtr msg) -> void
        {
          int now_ns = std::chrono::nanoseconds(_SC::now().time_since_epoch()).count();
          this->recv_jitters[this->count % num_bin] = now_ns - msg->time_sent_ns;
          this->count++;
        };
    sub_ = this->create_subscription<twmsgs::msg::Data>(topic_name, qos, callback_sub);
  }

  // number of subscribe
  int count;
  // recent receive jitters
  int recv_jitters[num_bin];

private:
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr sub_;
};

#endif  // SUBNODE_HPP_
