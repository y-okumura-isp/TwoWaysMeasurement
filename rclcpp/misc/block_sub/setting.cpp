#include "setting.hpp"

const std::string g_topic_name = "topic_name";
const rclcpp::QoS g_qos = rclcpp::QoS(1).best_effort();
const int max_subscription = 5;
const long period_ms = 50;
const long sleep_ms = 3;
