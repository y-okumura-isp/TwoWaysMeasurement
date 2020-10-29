cmake_minimum_required(VERSION 3.5)
project(multi_sub)

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
find_package(twmsgs REQUIRED)
find_package(tlsf_cpp REQUIRED)
find_package(rttest)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(../../)
set(TW_COMMON
  ../../common/tw_utils.cpp
  ../../common/tw_node_options.cpp
  setting.cpp
  )

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

set(DEPENDS
  rclcpp
  twmsgs
  tlsf_cpp
  rttest
  )

# one exec, one node, ping
set(MAIN_SUB main_sub)
add_executable(${MAIN_SUB} main_multi_sub.cpp ${TW_COMMON})
ament_target_dependencies(${MAIN_SUB} ${DEPENDS})

set(MAIN_PUB main_pub)
add_executable(${MAIN_PUB} main_pub.cpp ${TW_COMMON})
ament_target_dependencies(${MAIN_PUB} ${DEPENDS})


install(TARGETS
  ${MAIN_SUB}
  ${MAIN_PUB}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
