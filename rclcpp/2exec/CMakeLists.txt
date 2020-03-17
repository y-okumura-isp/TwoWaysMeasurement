cmake_minimum_required(VERSION 3.5)
project(tw_rclcpp_2exec_topic)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(twmsgs REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(tlsf_cpp)
find_package(rttest)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(TW_COMMON
  ../common/tw_utils.cpp
  ../common/two_ways_node.cpp
  ../common/tw_node_options.cpp
  ../common/two_ways_service_node.cpp
  )

set(DEPENDS
  rclcpp
  std_msgs
  twmsgs
  tlsf_cpp
  rosidl_generator_cpp
  rttest
  )

# two exec, two node, ping
set(TEP tw_2exec_pub)
add_executable(${TEP} main_pub.cpp ${TW_COMMON})
ament_target_dependencies(${TEP} ${DEPENDS})

set(TES tw_2exec_sub)
add_executable(${TES} main_sub.cpp ${TW_COMMON})
ament_target_dependencies(${TES} ${DEPENDS})

# two exec, two node, ping-pong
set(TEP_PIPO tw_2exec_pub_ping_pong)
add_executable(${TEP_PIPO} main_pub_ping_pong.cpp ${TW_COMMON})
ament_target_dependencies(${TEP_PIPO} ${DEPENDS})

set(TES_PIPO tw_2exec_sub_ping_pong)
add_executable(${TES_PIPO} main_sub_ping_pong.cpp ${TW_COMMON})
ament_target_dependencies(${TES_PIPO} ${DEPENDS})

install(TARGETS
  ${TEP}
  ${TES}
  ${TEP_PIPO}
  ${TES_PIPO}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
