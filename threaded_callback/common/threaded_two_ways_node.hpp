#ifndef THREADED_TWO_WAYS_NODE_HPP
#define THREADED_TWO_WAYS_NODE_HPP

#include "../../../ROS2_ThreadedCallback/include/threaded_wall_timer.hpp"

#include "../../rclcpp/common/tw_node_options.hpp"
#include "../../rclcpp/common/tw_utils.hpp"

class ThreadedTwoWaysNode : public rclcpp::Node
{
public:
  explicit ThreadedTwoWaysNode(
      const std::string name,
      const std::string namespace_,
      const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~ThreadedTwoWaysNode()
  {}

  void setup_ping_publisher();

private:
  rclcpp::TimerBase::SharedPtr ping_timer_;
  std::unique_ptr<ThreadedWallTimer> ping_helper_;

  const TwoWaysNodeOptions & tw_options_;
};

#endif // THREADED_TWO_WAYS_NODE_HPP
