#ifndef PUBNODE_HPP_
#define PUBNODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "twmsgs/msg/data.hpp"
#include "TwoWaysNode.hpp"

class PubNode : public TwoWaysNode
{
public:
  PubNode(const TwoWaysNodeOptions & tw_options,
          const rclcpp::NodeOptions & options)
      : TwoWaysNode(tw_options.node_name_pub, tw_options.namespace_, tw_options, options)
  {
    this->setup_ping_publisher();
  }
};

#endif  // PUBNODE_HPP_
