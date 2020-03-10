#ifndef SUBNODE_HPP_
#define SUBNODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"
#include "two_ways_node.hpp"

class SubNode : public TwoWaysNode
{
public:
  SubNode(const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
          const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
      : TwoWaysNode(tw_options.node_name_sub, tw_options.namespace_, tw_options, options)
  {
    this->setup_ping_subscriber();
  }
};

#endif  // SUBNODE_HPP_
