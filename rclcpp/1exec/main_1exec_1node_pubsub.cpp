#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using _SC = std::chrono::steady_clock;
using TIME_POINT = std::chrono::time_point<std::chrono::steady_clock>;

const char * node_name = "one_node_pub_sub";
const char * namespace_ = "ns";
const char * topic_name = "ping";
const int qos = 10;
const int period_ns = 100 * 1000 * 1000; // wake up period[ns]
const int num_bin = 10000; // number of time reports.

class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode(const rclcpp::NodeOptions & options)
      : Node(node_name, options), count(0)
  {
    // sub
    auto callback_sub =
        [this](const std_msgs::msg::String::SharedPtr msg) -> void
        {
          (void)msg;

          auto now = _SC::now();
          auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(now - this->time_send);
          this->recv_jitters[this->count % num_bin] = diff.count();
        };
    sub_ = this->create_subscription<std_msgs::msg::String>(topic_name, qos, callback_sub);

    // pub
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, qos);
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
          std_msgs::msg::String msg;
          msg.data = "HelloWorld" + std::to_string(this->count);
          // std::cout << "send: " < msg.data.c_str() << std::endl;
          this->count++;
          this->time_send = _SC::now();
          pub_->publish(msg);
        };

    // set timer
    this->epoch = _SC::now();
    timer = this->create_wall_timer(std::chrono::nanoseconds(period_ns), callback_pub);
  }

  void print_result() const;

private:
  int count;

  TIME_POINT epoch;
  int wakeup_jitters[num_bin];
  int recv_jitters[num_bin];

  TIME_POINT time_send;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  void _report_jitter(const std::string name, const int *bgn, int num_loop) const;
};

void PubSubNode::print_result() const
{
  std::cout << "count: " << this->count << std::endl;
  int num_loop = std::min(this->count, num_bin);

  // print wakeup jitter
  _report_jitter("wakeup_jitters", this->wakeup_jitters, num_loop);

  // print wakeup jitter
  _report_jitter("recv_jitters", this->recv_jitters, num_loop);
}

void PubSubNode::_report_jitter(const std::string name, const int bgn[], int num_loop) const
{
  std::vector<int> vec;
  std::cout << name << std::endl;
  // print wakeup jitter
  for(int i=0; i<num_loop; i++) {
    std::cout << bgn[i] << " ";
    vec.push_back(bgn[i]);
  }
  std::cout << std::endl;
  std::cout << "avg jitter: " << std::accumulate(vec.begin(), vec.end(), 0.0) / num_loop << std::endl;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions node_options;
  // TODO: node_options.allocator = ...;
  // TODO: node_options.use_intra_process_comms = ...;
  auto n = std::make_shared<PubSubNode>(node_options);

  exec.add_node(n);
  exec.spin();
  exec.remove_node(n);

  n->print_result();

  rclcpp::shutdown();
}
