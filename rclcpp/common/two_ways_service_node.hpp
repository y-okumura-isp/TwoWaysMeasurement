#ifndef TWO_WAYS_SERVICE_NODE_HPP_
#define TWO_WAYS_SERVICE_NODE_HPP_

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "twmsgs/srv/data.hpp"
#include "../common/tw_node_options.hpp"

class TwoWaysServiceNode : public rclcpp::Node
{
  using _SC = std::chrono::system_clock;
  using TIME_POINT = std::chrono::time_point<std::chrono::system_clock>;

public:
  TwoWaysServiceNode(
      const std::string name,
      const std::string namespace_,
      const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
      : Node(name, namespace_, options), tw_options_(tw_options) {}

  bool setup_ping_client();
  bool setup_ping_service();

protected:
  const TwoWaysNodeOptions & tw_options_;

private:
  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Client<twmsgs::srv::Data>::SharedPtr ping_client_;
  rclcpp::Service<twmsgs::srv::Data>::SharedPtr ping_server_;
  int ping_send_count_ = 0;
  int ping_recv_count_ = 0;
};

#endif  // TWO_WAYS_SERVICE_NODE_HPP_
