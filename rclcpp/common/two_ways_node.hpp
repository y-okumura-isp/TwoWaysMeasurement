#ifndef TWO_WAYS_NODE_HPP_
#define TWO_WAYS_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "twmsgs/msg/data.hpp"
#include "tw_node_options.hpp"

class TwoWaysNode : public rclcpp::Node
{
  using _SC = std::chrono::system_clock;
  using TIME_POINT = std::chrono::time_point<std::chrono::system_clock>;

public:
  explicit TwoWaysNode(
      const std::string name,
      const std::string namespace_,
      const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
      : Node(name, namespace_, options),
        ping_pub_count_(0), ping_wakeup_jitters_(nullptr),
        ping_sub_count_(0), ping_sub_jitters_(nullptr),
        tw_options_(tw_options)
  {
    this->ping_wakeup_jitters_ = new int[tw_options_.num_bin];
    this->ping_sub_jitters_ = new int[tw_options_.num_bin];
  }

  virtual ~TwoWaysNode()
  {
    delete[] this->ping_wakeup_jitters_;
    delete[] this->ping_sub_jitters_;
  }

  // number of ping publish
  int ping_pub_count_;
  // recent wakeup jitters
  int *ping_wakeup_jitters_;
  // ping epoch
  TIME_POINT ping_epoch_;

  // number of ping subscribe
  int ping_sub_count_;
  // recent receive jitters
  int *ping_sub_jitters_;

protected:
  const TwoWaysNodeOptions & tw_options_;

  void setup_ping_publisher();
  void setup_ping_subscriber();

  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Publisher<twmsgs::msg::Data>::SharedPtr ping_pub_;
  rclcpp::Subscription<twmsgs::msg::Data>::SharedPtr ping_sub_;

};

#endif  // TWO_WAYS_NODE_HPP_
