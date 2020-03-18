#ifndef CYCLIC_COMPONENT_HPP_
#define CYCLIC_COMPONENT_HPP_

#include <chrono>
#include <fstream>

// #include <composition/visibility_control.h>
#include <rclcpp/rclcpp.hpp>

namespace cyclic_component
{

class CyclicNode : public rclcpp::Node
{
  using SC = std::chrono::system_clock;
  using TIME_POINT = std::chrono::time_point<SC>;

public:
  // COMPOSITION_PUBLIC
  explicit CyclicNode(const rclcpp::NodeOptions & options);
  ~CyclicNode();

private:
  TIME_POINT epoch_;
  TIME_POINT last_fin_;

  int period_ns_;
  int sleep_ns_;
  int count_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string fout_;
  std::ofstream os_;

  struct timespec epoch_ts_;
  struct timespec last_fin_ts_;
  struct timespec expect_ts_;
  struct timespec period_ns_ts_;

  int diff_ns(TIME_POINT lhs, TIME_POINT rhs);
  int tp2ns(TIME_POINT tp);
};

}  // namespace cyclic_component

#endif  // CYCLIC_COMPONENT_HPP_
