#include <rttest/utils.hpp>
#include "two_ways_service_node.hpp"

using namespace std::chrono_literals;

bool TwoWaysServiceNode::setup_ping_client()
{
  ping_client_ = create_client<twmsgs::srv::Data>(tw_options_.service_name);

  // wait server
  while (!ping_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      std::cout << "Interrupted while waiting for the service. Exiting." << std::endl;
      return false;
    }
    std::cout << "service not available, waiting again..." << std::endl;
  }
  std::cout << "service found" << std::endl;
  req = std::make_shared<twmsgs::srv::Data::Request>();

  auto send_request =
      [this]() -> void
      {
        struct timespec time_wake_ts;
        getnow(&time_wake_ts);

        // calc wakeup jitter
        int64_t wake_latency = 0;
        if (rcl_timer_get_time_since_last_call(
                &(*this->ping_timer_->get_timer_handle()), &wake_latency) != RCL_RET_OK) {
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
        ping_send_count_++;
        last_wake_ts_ = time_wake_ts;

        // send request
        auto time_wake_ns = _timespec_to_long(&time_wake_ts);
        req->time_sent_ns = time_wake_ns;
        req->data = ping_send_count_;

        // define pong callback
        using ServiceResponseFuture =
            rclcpp::Client<twmsgs::srv::Data>::SharedFuture;
        auto response_callback =
            [this, time_wake_ns](ServiceResponseFuture future)
            {
              struct timespec time_get_pong_ts;
              getnow(&time_get_pong_ts);
              auto time_get_pong_ns = _timespec_to_long(&time_get_pong_ts);

              // std::cout << "ping-pong" << std::endl;
              auto result = future.get();
              auto pong_sent_ns = result->time_sent_ns;

              // pong latency
              pong_trans_report_.add(time_get_pong_ns - pong_sent_ns);

              ping_pong_report_.add(time_get_pong_ns - time_wake_ns);
            };

        // send ping
        ping_client_->async_send_request(req, response_callback);

        // fin
        if(ping_send_count_ == tw_options_.num_loops_) {
          std::raise(SIGINT);
        }
      };

  getnow(&epoch_ts_);
  period_ts_.tv_sec = 0;
  period_ts_.tv_nsec = tw_options_.period_ns;
  add_timespecs(&epoch_ts_, &period_ts_, &expect_ts_);
  last_wake_ts_ = epoch_ts_;

  ping_timer_ = this->create_wall_timer(std::chrono::nanoseconds(tw_options_.period_ns), send_request);

  return true;
}

bool TwoWaysServiceNode::setup_ping_service()
{
  auto service_callback =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<twmsgs::srv::Data::Request> request,
             const std::shared_ptr<twmsgs::srv::Data::Response> response) -> void
      {
        // std::cout << "recv ping" << std::endl;
        (void) request_header;
        (void) request;
        ping_recv_count_++;

        struct timespec now;
        getnow(&now);
        auto now_ns = _timespec_to_long(&now);
        ping_sub_report_.add(now_ns - request->time_sent_ns);

        response->time_sent_ns = now_ns;
      };
  ping_server_ = create_service<twmsgs::srv::Data>(tw_options_.service_name, service_callback);
  return true;
}

