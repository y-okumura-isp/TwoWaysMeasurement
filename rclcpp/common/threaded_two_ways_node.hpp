#ifndef THREADED_TWO_WAYS_NODE_HPP
#define THREADED_TWO_WAYS_NODE_HPP

#include "../../../ROS2_ThreadedCallback/include/threaded_wall_timer.hpp"
#include "../../../ROS2_ThreadedCallback/include/threaded_subscriber.hpp"

#include "twmsgs/msg/data.hpp"
#include "../../rclcpp/common/tw_node_options.hpp"
#include "../../rclcpp/common/tw_utils.hpp"

class PingPublisherByTimer : public ThreadedWallTimer
{
  using MyMsg = twmsgs::msg::Data;

public:
  PingPublisherByTimer(
      const TwoWaysNodeOptions &tw_options,
      rclcpp::Node *node, const std::string &topic_name, const rclcpp::QoS qos,
      long period_ns, bool debug_print, uint64_t num_loops,
      size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1);

  void print_ping_wakeup_report() {
    ping_wakeup_report_.print("ping_wakeup");
  }
  void print_diff_wakeup_report() {
    diff_wakeup_report_.print("diff_wakeup");
  }
  void print_timer_callback_process_time_report() {
    timer_callback_process_time_report_.print("timer_callback");
  }

protected:
  void on_callback() override;

  void on_overrun() override;

private:
  rclcpp::Publisher<MyMsg>::SharedPtr ping_pub_;
  uint64_t ping_pub_count_;
  const std::string topic_;
  MyMsg msg_;

  struct timespec epoch_ts_;
  struct timespec period_ts_;
  struct timespec expect_ts_;
  struct timespec last_wake_ts_;

  // wakeup jitter report
  JitterReportWithSkip ping_wakeup_report_;
  // wakeup jitter from last wakeup
  JitterReportWithSkip diff_wakeup_report_;
  // timer callback process time report
  JitterReportWithSkip timer_callback_process_time_report_;

  bool debug_print_;
  uint64_t num_loops_;

  int64_t get_now_int64() {
    struct timespec now_ts;
    getnow(&now_ts);
    return _timespec_to_long(&now_ts);
  }
};

class PingSubscription : public ThreadedSubscription<twmsgs::msg::Data::UniquePtr, twmsgs::msg::Data>
{
  using MyMsg = twmsgs::msg::Data;

public:
  PingSubscription(
      const TwoWaysNodeOptions &tw_options,
      rclcpp::Node *node,
      bool send_pong, bool debug_print,
      size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1);

  void print_ping_sub_report() {
    ping_sub_report_.print("ping_sub");
    std::cout << "ping_drop: " << ping_drop << std::endl;
    std::cout << "ping_drop_gap_: " << ping_drop_gap_ << std::endl;
    std::cout << "ping_argdrop: < " << ping_argdrop_ << std::endl;
    std::cout << "ping_late: " << ping_late << std::endl;
    std::cout << "ping_sub_count_: " << ping_sub_count_ << std::endl;
    std::cout << "ping_argmax: " << ping_argmax_ << std::endl << std::endl;
  }
  void print_ping_callback_process_time_report() {
    ping_callback_process_time_report_.print("ping_callback");
  }

protected:
  void on_callback() override;

  void on_overrun() override;

private:
  // number of ping subscribe
  int ping_sub_count_;

  rclcpp::Publisher<MyMsg>::SharedPtr pong_pub_;

  bool send_pong_;

  // how many times ping drop
  uint64_t ping_drop;
  uint64_t ping_drop_gap_;
  uint64_t ping_argmax_;
  uint64_t ping_argdrop_;
  // how many times ping late
  uint64_t ping_late;

  // sub jitter report
  JitterReportWithSkip ping_sub_report_;
  // ping callback process time report
  JitterReportWithSkip ping_callback_process_time_report_;

  bool debug_print_;

  int64_t get_now_int64() {
    struct timespec now_ts;
    getnow(&now_ts);
    return _timespec_to_long(&now_ts);
  }
};

class PongSubscription : public ThreadedSubscription<twmsgs::msg::Data::UniquePtr, twmsgs::msg::Data>
{
  using MyMsg = twmsgs::msg::Data;

public:
  PongSubscription(
      const TwoWaysNodeOptions &tw_options,
      rclcpp::Node *node,
      size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1);

  void print_pong_sub_report() {
    pong_sub_report_.print("pong_sub");
    std::cout << "pong_drop: " << pong_drop << std::endl;
    std::cout << "pong_drop_gap_: " << pong_drop_gap_ << std::endl;
    std::cout << "pong_argdrop: " << pong_argdrop_ << std::endl;
    std::cout << "pong_late: " << pong_late << std::endl;
    std::cout << "pong_sub_count_: " << pong_sub_count_ << std::endl;
    std::cout << "pong_argmax: " << pong_argmax_ << std::endl << std::endl;
  }
  void print_ping_pong_report() {
    ping_pong_report_.print("ping_pong");
  }
  void print_pong_callback_process_time_report(){
    pong_callback_process_time_report_.print("pong_callback");
  }

protected:
  void on_callback() override;

  void on_overrun() override;

private:
  // number of pong subscribe
  int pong_sub_count_;

  // how many times pong drop
  uint64_t pong_drop;
  uint64_t pong_drop_gap_;
  uint64_t pong_argmax_;
  uint64_t pong_argdrop_;
  // how many times pong late
  uint64_t pong_late;

  // sub jitter report
  JitterReportWithSkip pong_sub_report_;
  // ping-pong jitter report
  JitterReportWithSkip ping_pong_report_;
  // pong callback process time report
  JitterReportWithSkip pong_callback_process_time_report_;
};

class ThreadedTwoWaysNode : public rclcpp::Node
{
  using MyMsg = twmsgs::msg::Data;

public:
  explicit ThreadedTwoWaysNode(
      const std::string name,
      const std::string namespace_,
      const TwoWaysNodeOptions & tw_options = TwoWaysNodeOptions(),
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~ThreadedTwoWaysNode()
  {}

  void setup_ping_publisher();
  void setup_ping_subscriber(bool send_pong=false);
  void setup_pong_subscriber();

  void print_ping_wakeup_report() {
    ping_helper_->print_ping_wakeup_report();
  }
  void print_diff_wakeup_report() {
    ping_helper_->print_diff_wakeup_report();
  }
  void print_ping_sub_report() {
    ping_sub_helper_->print_ping_sub_report();
  }
  void print_pong_sub_report() {
    pong_sub_helper_->print_pong_sub_report();
  }
  void print_ping_pong_report() {
    pong_sub_helper_->print_ping_pong_report();
  }
  void print_timer_callback_process_time_report() {
    ping_helper_->print_timer_callback_process_time_report();
  }
  void print_ping_callback_process_time_report() {
    ping_sub_helper_->print_ping_callback_process_time_report();
  }
  void print_pong_callback_process_time_report() {
    pong_sub_helper_->print_pong_callback_process_time_report();
  }

private:
  rclcpp::TimerBase::SharedPtr ping_timer_;
  rclcpp::Subscription<MyMsg>::SharedPtr ping_sub_;
  rclcpp::Subscription<MyMsg>::SharedPtr pong_sub_;

  std::unique_ptr<PingPublisherByTimer> ping_helper_;
  std::unique_ptr<PingSubscription> ping_sub_helper_;
  std::unique_ptr<PongSubscription> pong_sub_helper_;

  const TwoWaysNodeOptions & tw_options_;
};

#endif // THREADED_TWO_WAYS_NODE_HPP
