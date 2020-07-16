#ifndef MAIN_PING_PONG_HPP
#define MAIN_PING_PONG_HPP

#include <typeinfo>

#include <rclcpp/rclcpp.hpp>
#include "../common/tw_node_options.hpp"

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

template<typename NodeClass>
class Runner_1e1n : public Runner
{
public:
  Runner_1e1n(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {
    std::cout << "runner = 1e1n " << typeid(NodeClass).name() << std::endl;
  }

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    n_ = std::make_shared<NodeClass>("node_1e1n", "ns", tw_options, node_options);
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
    n_->print_timer_callback_process_time_report();
    n_->print_ping_callback_process_time_report();
    n_->print_pong_callback_process_time_report();
  }

private:
  typename std::shared_ptr<NodeClass> n_;
};

template<typename NodeClass>
class Runner_1e2n : public Runner
{
public:
  Runner_1e2n(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {
      std::cout << "runner = 1e2n "  << typeid(NodeClass).name() << std::endl;
  }

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    npub_ = std::make_shared<NodeClass>("pub", "ns", tw_options, node_options);
    nsub_ = std::make_shared<NodeClass>("sub", "ns", tw_options, node_options);

    npub_->setup_ping_publisher();
    npub_->setup_pong_subscriber();
    nsub_->setup_ping_subscriber(true);

    exec_->add_node(npub_);
    exec_->add_node(nsub_);
  }

  void cleanup() override {
    exec_->remove_node(nsub_);
    exec_->remove_node(npub_);
  }

  void report() override {
     npub_->print_ping_wakeup_report();
     npub_->print_diff_wakeup_report();
     nsub_->print_ping_sub_report();
     npub_->print_pong_sub_report();
     npub_->print_ping_pong_report();
     npub_->print_timer_callback_process_time_report();
     nsub_->print_ping_callback_process_time_report();
     npub_->print_pong_callback_process_time_report();
  }

private:
  std::shared_ptr<NodeClass> npub_, nsub_;
};

template<typename NodeClass>
class Runner_2e_ping : public Runner
{
public:
  Runner_2e_ping(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {
      std::cout << "runner = 2e_ping " << typeid(NodeClass).name() << std::endl;
  }

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    npub_ = std::make_shared<NodeClass>("main_pub_ping_pong", "ns", tw_options, node_options);

    npub_->setup_ping_publisher();
    npub_->setup_pong_subscriber();
    exec_->add_node(npub_);
  }

  void cleanup() override {
    exec_->remove_node(npub_);
  }

  void report() override {
    npub_->print_ping_wakeup_report();
    npub_->print_diff_wakeup_report();
    npub_->print_pong_sub_report();
    npub_->print_ping_pong_report();
    npub_->print_timer_callback_process_time_report();
    npub_->print_pong_callback_process_time_report();
  }

private:
  std::shared_ptr<NodeClass> npub_;
};

template<typename NodeClass>
class Runner_2e_pong : public Runner
{
public:
  Runner_2e_pong(rclcpp::executor::Executor::SharedPtr e):
      Runner(e) {
      std::cout << "runner = 2e_pong " << typeid(NodeClass).name() << std::endl;
  }

  void setup(const TwoWaysNodeOptions &tw_options) override {
    rclcpp::NodeOptions node_options;
    tw_options.set_node_options(node_options);
    nsub_ = std::make_shared<NodeClass>("main_sub_ping_pong", "ns", tw_options, node_options);

    nsub_->setup_ping_subscriber(true);
    exec_->add_node(nsub_);
  }

  void cleanup() override {
    exec_->remove_node(nsub_);
  }

  void report() override {
    nsub_->print_ping_sub_report();
    nsub_->print_ping_callback_process_time_report();
  }

private:
  std::shared_ptr<NodeClass> nsub_;
};

#endif // MAIN_PING_PONG_HPP
