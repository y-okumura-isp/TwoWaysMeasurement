#include <rclcpp/rclcpp.hpp>

#include "../common/tw_node_options.hpp"
#include "../common/tw_utils.hpp"
#include "../common/two_ways_service_node.hpp"

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;
  rclcpp::init(argc, argv);

  TwoWaysNodeOptions tw_options;
  auto exec = tw_options.get_executor();

  rclcpp::NodeOptions node_options;
  tw_options.set_node_options(node_options);

  auto node = std::make_shared<TwoWaysServiceNode>(tw_options.node_name, tw_options, node_options);
  if (!node->setup_ping_service()) {
     std::cerr << "cannot setup service" << std::endl;
  }
  if (!node->setup_ping_client()) {
    std::cerr << "cannot setup client" << std::endl;
  }

  exec->add_node(node);
  exec->spin();
  exec->remove_node(node);

  std::cout << "fin" << std::endl;
  node->print_ping_wakeup_report();
  node->print_ping_sub_report();
  node->print_pong_trans_report();
  node->print_ping_pong_report();
}

// Here is a minimum sample of server + (client + timer)
/*
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  std::cout << "Press C-c to quit" << std::endl;
  const std::string service_name = "ping";

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test");

  // service
  auto service_callback =
      [](const std::shared_ptr<rmw_request_id_t> request_header,
         const std::shared_ptr<twmsgs::srv::Data::Request> request,
         const std::shared_ptr<twmsgs::srv::Data::Response> response) -> void
      {
        (void) request_header;
        (void) request;
        std::cout << "service receives" << std::endl;
        response->time_sent_ns = 1;
      };
  auto ping_server = node->create_service<twmsgs::srv::Data>(service_name, service_callback);

  // client
  auto ping_client = node->create_client<twmsgs::srv::Data>(service_name);
  std::cout << "create_client" << std::endl;
  while (!ping_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      std::cout << "Interrupted while waiting for the service. Exiting." << std::endl;
      return false;
    }
    std::cout << "service not available, waiting again..." << std::endl;
  }
  std::cout << "service found" << std::endl;

  auto send_request =
      [ping_client]() -> void
      {
        // clients send request
        auto req = std::make_shared<twmsgs::srv::Data::Request>();
        req->time_sent_ns = 0;
        req->data = 0;

        using ServiceResponseFuture =
            rclcpp::Client<twmsgs::srv::Data>::SharedFuture;
        auto response_callback =
            [](ServiceResponseFuture future)
            {
              auto result = future.get();
              std::cout << "client: " << result->time_sent_ns << std::endl;
            };
        ping_client->async_send_request(req, response_callback);
      };
  auto ping_timer = node->create_wall_timer(1s, send_request);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(node);
  exec->spin();
  exec->remove_node(node);
}
*/
