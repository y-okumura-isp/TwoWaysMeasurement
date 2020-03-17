cmake_minimum_required(VERSION 3.5)
project(tw_rclcpp)

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
  common/tw_utils.cpp
  common/two_ways_node.cpp
  common/tw_node_options.cpp
  common/two_ways_service_node.cpp
  )

set(DEPENDS
  rclcpp
  std_msgs
  twmsgs
  tlsf_cpp
  rosidl_generator_cpp
  rttest
  )

# one exec, one node, ping
set(OEON tw_1exec_1node)
add_executable(${OEON} 1exec/main_1exec_1node_pubsub.cpp ${TW_COMMON})
ament_target_dependencies(${OEON} ${DEPENDS})

# one exec, one node, ping-pong
set(OEON_PIPO tw_1exec_1node_ping_pong)
add_executable(${OEON_PIPO} 1exec/main_1exec_1node_ping_pong.cpp ${TW_COMMON})
ament_target_dependencies(${OEON_PIPO} ${DEPENDS})

# one exec, two node, ping
set(OETN tw_1exec_2node)
add_executable(${OETN} 1exec/main_1exec_2node_pubsub.cpp ${TW_COMMON})
ament_target_dependencies(${OETN} ${DEPENDS})

# one exec, two node, ping-pong
set(OETN_PIPO tw_1exec_2node_ping_pong)
add_executable(${OETN_PIPO} 1exec/main_1exec_2node_ping_pong.cpp ${TW_COMMON})
ament_target_dependencies(${OETN_PIPO} ${DEPENDS})

# two exec, two node, ping
set(TEP tw_2exec_pub)
add_executable(${TEP} 2exec/main_pub.cpp ${TW_COMMON})
ament_target_dependencies(${TEP} ${DEPENDS})

set(TES tw_2exec_sub)
add_executable(${TES} 2exec/main_sub.cpp ${TW_COMMON})
ament_target_dependencies(${TES} ${DEPENDS})

# two exec, two node, ping-pong
set(TEP_PIPO tw_2exec_pub_ping_pong)
add_executable(${TEP_PIPO} 2exec/main_pub_ping_pong.cpp ${TW_COMMON})
ament_target_dependencies(${TEP_PIPO} ${DEPENDS})

set(TES_PIPO tw_2exec_sub_ping_pong)
add_executable(${TES_PIPO} 2exec/main_sub_ping_pong.cpp ${TW_COMMON})
ament_target_dependencies(${TES_PIPO} ${DEPENDS})

# one exec, one node, service
set(OEON_SRV tw_1exec_1node_srv)
add_executable(${OEON_SRV} 1exec/main_1exec_1node_service.cpp ${TW_COMMON})
ament_target_dependencies(${OEON_SRV} ${DEPENDS})

# one exec, two node, service
set(OETN_SRV tw_1exec_2node_srv)
add_executable(${OETN_SRV} 1exec/main_1exec_2node_service.cpp ${TW_COMMON})
ament_target_dependencies(${OETN_SRV} ${DEPENDS})

# two exec, server
set(TE_SERVER tw_2exec_server)
add_executable(${TE_SERVER} 2exec/main_server.cpp ${TW_COMMON})
ament_target_dependencies(${TE_SERVER} ${DEPENDS})

# two exec, client
set(TE_CLIENT tw_2exec_client)
add_executable(${TE_CLIENT} 2exec/main_client.cpp ${TW_COMMON})
ament_target_dependencies(${TE_CLIENT} ${DEPENDS})

install(TARGETS
  ${OEON}
  ${OEON_PIPO}
  ${OETN}
  ${OETN_PIPO}
  ${TEP}
  ${TES}
  ${TEP_PIPO}
  ${TES_PIPO}
  ${OEON_SRV}
  ${OETN_SRV}
  ${TE_SERVER}
  ${TE_CLIENT}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
