#ifndef CYCLIC_COMPONENT_HPP_
#define CYCLIC_COMPONENT_HPP_

#include <fstream>

#include <rclcpp/rclcpp.hpp>

namespace cyclic_component
{

class CyclicNode : public rclcpp::Node
{
public:
  explicit CyclicNode(const rclcpp::NodeOptions & options);
  ~CyclicNode();

private:
  int period_ns_;
  int sleep_ns_;
  int count_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string fout_;
  std::ofstream os_;

  struct timespec epoch_ts_;
  struct timespec last_wakeup_;
  struct timespec last_fin_ts_;
  struct timespec expect_ts_;
  struct timespec period_ns_ts_;
};

}  // namespace cyclic_component

#endif  // CYCLIC_COMPONENT_HPP_
