#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

namespace {

nav_msgs::msg::OccupancyGrid::UniquePtr initializeOccupancyGrid()
{
    auto gridMsg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

    gridMsg->header.stamp = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
    gridMsg->info.resolution = 0.35;
    gridMsg->info.width = 500;
    gridMsg->info.height = 500;
    gridMsg->info.origin = geometry_msgs::msg::Pose{ };
    gridMsg->info.origin.position.x = -gridMsg->info.resolution * static_cast<double>(100);
    gridMsg->info.origin.position.y = -gridMsg->info.resolution * static_cast<double>(250);

    gridMsg->info.origin.orientation.w = 1.0;
    gridMsg->info.origin.orientation.x = 0.0;
    gridMsg->info.origin.orientation.y = 0.0;
    gridMsg->info.origin.orientation.z = 0.0;

    gridMsg->data.resize(gridMsg->info.width * gridMsg->info.height);

    return gridMsg;
}

nav_msgs::msg::Odometry::UniquePtr initializeOdometry()
{
    auto odomMsg = std::make_unique<nav_msgs::msg::Odometry>();

    odomMsg->header.stamp = rclcpp::Clock{RCL_SYSTEM_TIME}.now();

    return odomMsg;
}

}

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;

    rclcpp::init(argc, argv, rclcpp::InitOptions());

    auto node = std::make_shared<rclcpp::Node>("simple_server");

    auto callbackGroup = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto subOpts = rclcpp::SubscriptionOptions{ };

    subOpts.callback_group = callbackGroup;

    auto gridPub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("simple_server/occupancy_grid", 1);

    auto odomPub = node->create_publisher<nav_msgs::msg::Odometry>("simple_server/odometry", 1);

    auto executor = rclcpp::executors::SingleThreadedExecutor{ };

    executor.add_node(node);

    while (rclcpp::ok())
    {
        auto gridMsg = initializeOccupancyGrid();

        if (gridMsg)
            gridPub->publish(std::move(gridMsg));

        auto odomMsg = initializeOdometry();

        if (odomMsg)
            odomPub->publish(std::move(odomMsg));

        std::this_thread::sleep_for(20ms);
    }

    return 0;
}
