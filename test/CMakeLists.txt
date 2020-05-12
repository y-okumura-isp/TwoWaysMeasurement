cmake_minimum_required(VERSION 3.5)
project(tw_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_gtest REQUIRED)
ament_find_gtest()

include_directories(../rclcpp/common)
# add_executable(
#   test_jitter_report
#   test_jitter_report.cpp
#   ../rclcpp/common/tw_utils.cpp)
# target_link_libraries(test_jitter_report gtest_main)
# add_test(NAME test_jitter_report COMMAND test_jitter_report)
ament_add_gtest(test_jitter_report
  test_jitter_report.cpp
  ../rclcpp/common/tw_utils.cpp)
