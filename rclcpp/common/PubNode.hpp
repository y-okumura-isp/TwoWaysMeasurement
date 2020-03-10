#ifndef PUBNODE_HPP_
#define PUBNODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"
#include "setting.h"

class PubNode: public rclcpp::Node
{
  using _SC = std::chrono::system_clock;
  using TIME_POINT = std::chrono::time_point<std::chrono::system_clock>;

public:
  PubNode(const rclcpp::NodeOptions & options)
      : Node(node_name_pub, options), count(0)
  {
    // pub
    pub_ = this->create_publisher<twmsgs::msg::Data>(topic_name, qos);
    auto callback_pub =
        [this]() -> void
        {
          // calc timer jitter
          TIME_POINT now = _SC::now();
          auto expect = this->epoch + std::chrono::nanoseconds(period_ns) * (this->count + 1);
          auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(now - expect);
          // std::cout << "timer latency: " << diff.count() << "[ns]" << std::endl;;
          this->wakeup_jitters[this->count % num_bin] = diff.count();

          // pub
          twmsgs::msg::Data msg;
          msg.data = "HelloWorld" + std::to_string(this->count);
          msg.time_sent_ns = std::chrono::nanoseconds(_SC::now().time_since_epoch()).count();
          pub_->publish(msg);

          this->count++;
        };

    // set timer
    this->epoch = _SC::now();
    timer = this->create_wall_timer(std::chrono::nanoseconds(period_ns), callback_pub);
  }

  // number of publishes
  int count;
  // recent wakeup jitters
  int wakeup_jitters[num_bin];

private:
  TIME_POINT epoch;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr pub_;
};

#endif  // PUBNODE_HPP_
