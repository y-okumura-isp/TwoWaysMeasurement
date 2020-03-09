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

const char * node_name_pub = "one_node_sub";
const char * node_name_sub = "one_node_pub";
const char * namespace_ = "ns";
const char * topic_name = "ping";
const int qos = 10;
const int period_ns = 100 * 1000 * 1000; // wake up period[ns]
const int num_bin = 10000; // number of time reports.

TIME_POINT time_send;

class PubNode: public rclcpp::Node
{
public:
  PubNode(const rclcpp::NodeOptions & options)
      : Node(node_name_pub, options), count(0)
  {
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
          time_send = _SC::now();
          pub_->publish(msg);
        };

    // set timer
    this->epoch = _SC::now();
    timer = this->create_wall_timer(std::chrono::nanoseconds(period_ns), callback_pub);
  }

  int count;
  int wakeup_jitters[num_bin];

private:
  TIME_POINT epoch;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  void _report_jitter(const std::string name, const int *bgn, int num_loop) const;
};

class SubNode : public rclcpp::Node
{
public:
  SubNode(const rclcpp::NodeOptions & options)
      : Node(node_name_sub, options), count(0)
  {
    // sub
    auto callback_sub =
        [this](const std_msgs::msg::String::SharedPtr msg) -> void
        {
          (void)msg;

          auto now = _SC::now();
          auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(now - time_send);
          this->recv_jitters[this->count % num_bin] = diff.count();
          this->count++;
        };
    sub_ = this->create_subscription<std_msgs::msg::String>(topic_name, qos, callback_sub);
  }

  void print_result() const;

  int recv_jitters[num_bin];

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  int count;
};

void _report_jitter(const std::string name, const int *bgn, int num_loop)
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

void print_result(const std::shared_ptr<PubNode> npub, const std::shared_ptr<SubNode> nsub)
{
  int num_loop = std::min(npub->count, num_bin);
  _report_jitter("wakeup_jitters", npub->wakeup_jitters, num_loop);
  _report_jitter("recv_jitters",   nsub->recv_jitters, num_loop);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  rclcpp::NodeOptions node_options;
  // TODO: node_options.allocator = ...;
  // TODO: node_options.use_intra_process_comms = ...;
  auto npub = std::make_shared<PubNode>(node_options);
  auto nsub = std::make_shared<SubNode>(node_options);

  exec.add_node(npub);
  exec.add_node(nsub);
  exec.spin();
  exec.remove_node(nsub);
  exec.remove_node(npub);

  print_result(npub, nsub);

  rclcpp::shutdown();
}
