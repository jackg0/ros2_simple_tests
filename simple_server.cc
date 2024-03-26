#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;

    rclcpp::init(argc, argv, rclcpp::InitOptions());

    auto node = std::make_shared<rclcpp::Node>("simple_server");

    //
    // service

    auto srv = node->create_service<std_srvs::srv::Trigger>(
        "simple_server/trigger",
        [](const std_srvs::srv::Trigger::Request::SharedPtr &, std_srvs::srv::Trigger::Response::SharedPtr res)
        {
            res->message = "service call successful";

            res->success = true;

            return true;
        },
        1);

    //
    // odometry subscription

    auto odomSub = node->create_subscription<nav_msgs::msg::Odometry>(
        "simple_server/odometry",
        5,
        [](const nav_msgs::msg::Odometry::UniquePtr &odom)
        {
            (void) odom;
        });

    // occupancy grid subscription

    auto gridSub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "simple_server/occupancy_grid",
        5,
        [](const nav_msgs::msg::OccupancyGrid::UniquePtr &grid)
        {
            (void) grid;
        });

    auto executor = rclcpp::executors::SingleThreadedExecutor{ };

    executor.add_node(node);

    while (rclcpp::ok())
    {
        RCLCPP_INFO(rclcpp::get_logger("simple_server"), "before spin");

        executor.spin_all(30ms);

        RCLCPP_INFO(rclcpp::get_logger("simple_server"), "after spin");
    }

    return 0;
}
