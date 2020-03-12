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
  )

# one exec, one node
set(OEON tw_1exec_1node)
add_executable(${OEON} 1exec/main_1exec_1node_pubsub.cpp ${TW_COMMON})
ament_target_dependencies(${OEON} ${DEPENDS})

# one exec, two node
set(OETN tw_1exec_2node)
add_executable(${OETN} 1exec/main_1exec_2node_pubsub.cpp ${TW_COMMON})
ament_target_dependencies(${OETN} ${DEPENDS})

# two exec, two node
set(TEP tw_2exec_pub)
add_executable(${TEP} 2exec/main_pub.cpp ${TW_COMMON})
ament_target_dependencies(${TEP} ${DEPENDS})

set(TES tw_2exec_sub)
add_executable(${TES} 2exec/main_sub.cpp ${TW_COMMON})
ament_target_dependencies(${TES} ${DEPENDS})

# one exec, one node, service
set(OEON_SRV tw_1exec_1node_srv)
add_executable(${OEON_SRV} 1exec/main_1exec_1node_service.cpp ${TW_COMMON})
ament_target_dependencies(${OEON_SRV} ${DEPENDS})

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
  ${OETN}
  ${TEP}
  ${TES}
  ${OEON_SRV}
  ${TE_SERVER}
  ${TE_CLIENT}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
