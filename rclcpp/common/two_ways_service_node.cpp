#include "two_ways_service_node.hpp"

using namespace std::chrono_literals;

int tp2ns(std::chrono::time_point<std::chrono::system_clock> tp)
{
  return std::chrono::nanoseconds(tp.time_since_epoch()).count();
}

int duration2ns(std::chrono::time_point<std::chrono::system_clock> to,
                std::chrono::time_point<std::chrono::system_clock> frm)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(to - frm).count();
}

bool TwoWaysServiceNode::setup_ping_client()
{
  auto period_ns = this->tw_options_.period_ns;

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

  auto send_request =
      [this, period_ns]() -> void
      {
        // std::cout << "send_request" << std::endl;
        // calc wakeup jitter
        TIME_POINT time_wake = _SC::now();
        auto expect = ping_epoch_ + std::chrono::nanoseconds(period_ns) * (ping_send_count_ + 1);
        ping_wakeup_report_.add(duration2ns(time_wake, expect));

        auto req = std::make_shared<twmsgs::srv::Data::Request>();

        // send request
        TIME_POINT time_sent = _SC::now();
        req->time_sent_ns = tp2ns(time_sent);
        req->data = ping_send_count_;

        using ServiceResponseFuture =
            rclcpp::Client<twmsgs::srv::Data>::SharedFuture;
        auto response_callback =
            [this, time_sent](ServiceResponseFuture future)
            {
              // std::cout << "ping-pong" << std::endl;
              auto result = future.get();
              TIME_POINT now = _SC::now();
              pong_trans_report_.add(tp2ns(now) - result->time_sent_ns);

              ping_pong_report_.add(duration2ns(now, time_sent));
            };

        ping_client_->async_send_request(req, response_callback);
        ping_send_count_++;
      };

  ping_epoch_ = _SC::now();
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

        auto now_ns = tp2ns(_SC::now());
        ping_sub_report_.add(now_ns - request->time_sent_ns);

        response->time_sent_ns = tp2ns(_SC::now());
      };
  ping_server_ = create_service<twmsgs::srv::Data>(tw_options_.service_name, service_callback);
  return true;
}

