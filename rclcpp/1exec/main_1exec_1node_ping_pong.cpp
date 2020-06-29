#include <chrono>
#include <memory>
#include <time.h>
#include <vector>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "../common/two_ways_node.hpp"
#include "../common/tw_utils.hpp"

const char * node_name = "one_node_ping_pong";

class Runner
{
public:
  Runner(rclcpp::executor::Executor::SharedPtr e):
      exec_(e) {}

  virtual void setup(const TwoWaysNodeOptions &tw_options) = 0;
  virtual void cleanup() = 0;
  virtual void report() = 0;

protected:
  rclcpp::executor::Executor::SharedPtr exec_;
};

class Runner_1e1n : public Runner
{
public:
  Runner_1e1n(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {}

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    n_ = std::make_shared<TwoWaysNode>("node_1e1n", "ns", tw_options, node_options);
    n_->setup_ping_publisher();
    n_->setup_ping_subscriber(true);
    n_->setup_pong_subscriber();

    exec_->add_node(n_);
  }

  void cleanup() override {
    exec_->remove_node(n_);
  }

  void report() override {
    n_->print_ping_wakeup_report();
    n_->print_diff_wakeup_report();
    n_->print_ping_sub_report();
    n_->print_pong_sub_report();
    n_->print_ping_pong_report();
  }

private:
  std::shared_ptr<TwoWaysNode> n_;
};

std::unique_ptr<Runner>
make_runner(RunType type, rclcpp::executor::Executor::SharedPtr e)
{
  std::unique_ptr<Runner> p(nullptr);
  switch(type) {
    case(E1N1): {
      p.reset(new Runner_1e1n(e));
      break;
    }
  }

  return p;
}

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;

  TwoWaysNodeOptions tw_options(argc, argv);

  if(lock_and_prefault_dynamic(tw_options.prefault_dynamic_size) < 0) {
    std::cerr << "lock_and_prefault_dynamic failed" << std::endl;
  }

  // setup child process scheule
  size_t priority = 0;
  int policy = 0;
  tw_options.get_child_thread_policy(priority, policy);
  set_sched_priority("child", priority, policy);

  rclcpp::init(argc, argv);
  auto exec = tw_options.get_executor();

  auto runner = make_runner(tw_options.run_type, exec);
  runner->setup(tw_options);

  // setup child process scheule
  tw_options.get_main_thread_policy(priority, policy);
  set_sched_priority("main", priority, policy);

  exec->spin();

  runner->cleanup();

  runner->report();

  rclcpp::shutdown();
}
