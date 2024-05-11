cmake_minimum_required(VERSION 3.20)
project(ros2_simple_tests VERSION 0.0.1 DESCRIPTION "simple pub/sub/service tests for ros2")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wcast-align
    -Wdisabled-optimization
    -Werror
    -Wextra
    -Wformat=2
    -Wfatal-errors
    -Wno-overloaded-virtual
    -Winit-self
    -Wmissing-include-dirs
    -Wreorder
    -Wsign-promo
    -Wswitch
    -Wunused
    -Wpedantic
    -Wno-unused-parameter # ros2 contains unused parameters
)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_logging_spdlog REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(spdlog REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

#
# simple_server

add_executable(simple_server simple_server.cc)

target_link_libraries(simple_server PUBLIC
    ${nav_msgs_TARGETS}
    rcl::rcl
    rcl_logging_spdlog::rcl_logging_spdlog
    rclcpp::rclcpp
    rmw::rmw
    ${std_srvs_TARGETS}
)

target_include_directories(simple_server PUBLIC
    ${nav_msgs_INCLUDE_DIRS}
    ${rcl_INCLUDE_DIRS}
    ${rcl_logging_spdlog_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
)

#
# simple_publisher

add_executable(simple_publisher simple_publisher.cc)

target_link_libraries(simple_publisher PUBLIC
    ${nav_msgs_TARGETS}
    rcl::rcl
    rcl_logging_spdlog::rcl_logging_spdlog
    rclcpp::rclcpp
    rmw::rmw
)

target_include_directories(simple_publisher PUBLIC
    ${nav_msgs_INCLUDE_DIRS}
    ${rcl_INCLUDE_DIRS}
    ${rcl_logging_spdlog_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
)

#
# throughput intraprocess

add_executable(throughput_intraprocess throughput_intraprocess.cc)

target_link_libraries(throughput_intraprocess PUBLIC
    rcl::rcl
    rcl_logging_spdlog::rcl_logging_spdlog
    rclcpp::rclcpp
    rmw::rmw
    spdlog::spdlog
    ${std_msgs_TARGETS}
)

target_include_directories(throughput_intraprocess PUBLIC
    ${rcl_INCLUDE_DIRS}
    ${rcl_logging_spdlog_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

#
# throughput publisher

add_executable(throughput_publisher throughput_publisher.cc)

target_link_libraries(throughput_publisher PUBLIC
    rcl::rcl
    rcl_logging_spdlog::rcl_logging_spdlog
    rclcpp::rclcpp
    rmw::rmw
    ${std_msgs_TARGETS}
)

target_include_directories(throughput_publisher PUBLIC
    ${rcl_INCLUDE_DIRS}
    ${rcl_logging_spdlog_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

#
# throughput subscriber

add_executable(throughput_subscriber throughput_subscriber.cc)

target_link_libraries(throughput_subscriber PUBLIC
    rcl::rcl
    rcl_logging_spdlog::rcl_logging_spdlog
    rclcpp::rclcpp
    rmw::rmw
    spdlog::spdlog
    ${std_msgs_TARGETS}
)

target_include_directories(throughput_subscriber PUBLIC
    ${rcl_INCLUDE_DIRS}
    ${rcl_logging_spdlog_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${rmw_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

install(
    TARGETS
    simple_publisher
    simple_server
    throughput_intraprocess
    throughput_publisher
    throughput_subscriber
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY scripts
    DESTINATION bin/${PROJECT_NAME}
)

ament_package()
