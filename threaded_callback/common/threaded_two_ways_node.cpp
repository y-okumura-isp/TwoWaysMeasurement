#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rttest/utils.h>

#include "threaded_two_ways_node.hpp"

#include "twmsgs/msg/data.hpp"
#include "../../rclcpp/common/tw_node_options.hpp"
#include "../../rclcpp/common/tw_utils.hpp"

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;

const std::string PERIOD_NS = "period_ns";
const std::string NUM_LOOPS = "num_loops";
const std::string DEBUG_PRINT = "debug_print";

using MyMsg = twmsgs::msg::Data;

class PingPublisherByTimer : public ThreadedWallTimer
{
public:
  PingPublisherByTimer(
      rclcpp::Node *node, const std::string &topic_name, const rclcpp::QoS qos,
      long period_ns, bool debug_print, uint64_t num_loops,
      size_t sched_priority=0, int policy=SCHED_OTHER, size_t core_id=1);

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

PingPublisherByTimer::PingPublisherByTimer(
    rclcpp::Node *node, const std::string &topic_name, const rclcpp::QoS qos,
    long period_ns, bool debug_print, uint64_t num_loops,
    size_t sched_priority, int policy, size_t core_id)
    : ThreadedWallTimer(sched_priority, policy, core_id),
      ping_pub_count_(0), topic_(topic_name),
      debug_print_(debug_print), num_loops_(num_loops)
{
  getnow(&epoch_ts_);
  period_ts_.tv_sec = 0;
  period_ts_.tv_nsec = period_ns;
  add_timespecs(&epoch_ts_, &period_ts_, &expect_ts_);
  last_wake_ts_ = epoch_ts_;
  ping_pub_ = node->create_publisher<MyMsg>(topic_name, qos);
}

void PingPublisherByTimer::on_callback()
{
  struct timespec time_wake_ts;
  getnow(&time_wake_ts);

  // calc wakeup jitter
  int64_t wake_latency = 0;
  if (rcl_timer_get_time_since_last_call(
          &(*this->timer_->get_timer_handle()), &wake_latency) != RCL_RET_OK) {
    return;
  }
  ping_wakeup_report_.add(wake_latency);

  // calc difference from last callback
  struct timespec diff_from_last_wakeup_ts;
  subtract_timespecs(&time_wake_ts, &last_wake_ts_, &diff_from_last_wakeup_ts);
  // minus period because distribution mean is period_ns.
  subtract_timespecs(&diff_from_last_wakeup_ts, &period_ts_, &diff_from_last_wakeup_ts);
  diff_wakeup_report_.add(_timespec_to_long(&diff_from_last_wakeup_ts));

  // prepere to next
  add_timespecs(&epoch_ts_, &period_ts_, &expect_ts_);
  ping_pub_count_++;
  last_wake_ts_ = time_wake_ts;

  // pub
  msg_.data = ping_pub_count_;
  // define original image
  for(size_t i=0; i< msg_.image.size(); i++) {
    msg_.image[i] = 0;
  }
  msg_.time_sent_ns = get_now_int64();
  ping_pub_->publish(msg_);

  if(debug_print_) {
    struct timespec time_print;
    getnow(&time_print);
    std::cout << "time_print.tv_sec: " << time_print.tv_sec << " "
              << "time_print.tv_nsec: " << time_print.tv_nsec << std::endl;
    std::cout << "sent ping id = " << ping_pub_count_
              << " @" << _timespec_to_long(&time_print)
              << " waked up @ " << _timespec_to_long(&time_wake_ts)
              << std::endl;
  }

  struct timespec time_exit;
  getnow(&time_exit);
  subtract_timespecs(&time_exit, &time_wake_ts, &time_exit);
  timer_callback_process_time_report_.add(_timespec_to_long(&time_exit));

  if(ping_pub_count_ == num_loops_) {
    std::raise(SIGINT);
  }
}

void PingPublisherByTimer::on_overrun()
{
}

/************************
 * ThreadedTwoWaysNode
 ************************/
ThreadedTwoWaysNode::ThreadedTwoWaysNode(
    const std::string name,
    const std::string namespace_,
    const TwoWaysNodeOptions & tw_options,
    const rclcpp::NodeOptions & options)
    : Node(name, namespace_, options),
      tw_options_(tw_options)
{
  declare_parameter(PERIOD_NS, 10 * 1000 * 1000);
  declare_parameter(NUM_LOOPS, 10000);
  declare_parameter(DEBUG_PRINT, false);
}


void ThreadedTwoWaysNode::setup_ping_publisher()
{
  auto topic_name = this->tw_options_.topic_name;
  auto qos = this->tw_options_.qos;
  auto period_ns = get_parameter(PERIOD_NS).get_value<int>();
  auto num_loops = get_parameter(NUM_LOOPS).get_value<int>();
  auto debug_print = get_parameter(DEBUG_PRINT).get_value<bool>();

  std::cout << PERIOD_NS   << ": " << period_ns << std::endl;
  std::cout << NUM_LOOPS   << ": " << num_loops << std::endl;
  std::cout << DEBUG_PRINT << ": " << (debug_print ? "true" : "false") << std::endl;

  ping_helper_ = std::make_unique<PingPublisherByTimer>(
      this, topic_name, qos,
      period_ns, debug_print, num_loops);
  this->ping_timer_ = ping_helper_->create_wall_timer(this, std::chrono::nanoseconds(period_ns));
}
