#ifndef SETTING_HPP_
#define SETTING_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>

extern const std::string g_topic_name;
extern const rclcpp::QoS g_qos;
extern const int max_subscription;
extern const long period_ms;
extern const long sleep_ms;

#endif  // SETTING_HPP_
