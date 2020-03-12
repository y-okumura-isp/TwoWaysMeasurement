#include "two_ways_service_node.hpp"

using namespace std::chrono_literals;

bool TwoWaysServiceNode::setup_ping_client()
{
  ping_client_ = create_client<twmsgs::srv::Data>(tw_options_.service_name);
  std::cout << "create_client" << std::endl;

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
      [this]() -> void
      {
        auto req = std::make_shared<twmsgs::srv::Data::Request>();

        // send request
        req->time_sent_ns = 0;
        req->data = ping_send_count_;

#if 1
        using ServiceResponseFuture =
            rclcpp::Client<twmsgs::srv::Data>::SharedFuture;
        auto response_callback =
            [this](ServiceResponseFuture future)
            {
              auto result = future.get();
              std::cout << "client: " << result->time_sent_ns << std::endl;
            };
        ping_client_->async_send_request(req, response_callback);
#else
        // This pass causes bellow runtime error
        //   terminate called after throwing an instance of 'std::runtime_error'
        //   what():  Node has already been added to an executor.
        auto result_future = this->ping_client_->async_send_request(req);

        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
          std::cerr << "oops, cannot get response" << std::endl;
          return;
        }
        auto result = result_future.get();
        std::cout << "client: " << result->time_sent_ns << std::endl;
#endif
        ping_send_count_++;
      };

  this->ping_timer_ = this->create_wall_timer(std::chrono::nanoseconds(tw_options_.period_ns), send_request);

  return true;
}

bool TwoWaysServiceNode::setup_ping_service()
{
  auto service_callback =
      [this](const std::shared_ptr<rmw_request_id_t> request_header,
             const std::shared_ptr<twmsgs::srv::Data::Request> request,
             const std::shared_ptr<twmsgs::srv::Data::Response> response) -> void
      {
        (void) request_header;
        (void) request;
        std::cout << "service" << std::endl;
        ping_recv_count_++;
        response->time_sent_ns = ping_recv_count_;
      };
  ping_server_ = create_service<twmsgs::srv::Data>(tw_options_.service_name, service_callback);
  std::cout << "create server" << std::endl;
  return true;
}

