#include <getopt.h>

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

struct Options
{
    size_t spinPeriodMilliseconds{30};
};

void showHelp(const std::string &name)
{
    std::cout << "usage: " << name << " OPTIONS\n"
        "OPTIONS:\n"
        " -t | --spin-period-milliseconds     spin period (ms); valid bounds: [1, 1000].\n"
        " -h | --help                         show this help.\n";
}

Options parseOpts(int argc, char **argv)
{
    static constexpr auto shortOpts = "t:h";
    static constexpr struct option longOpts[] = {
        { "spin-period-milliseconds", required_argument, nullptr, 't' },
        { "help", no_argument, nullptr, 'h' },
        { nullptr, 0, nullptr, 0 }
    };

    Options opts{ };

    for (int c = 0; (c = getopt_long(argc, argv, shortOpts, longOpts, nullptr)) >= 0; )
    {
        switch (c)
        {
            case 't':
                opts.spinPeriodMilliseconds = std::clamp<size_t>(std::stod(optarg), 1, 1000);
                break;
            case 'h':
                showHelp(argv[0]);
                std::exit(0);
            case '?':
                std::exit(1);
            default:
                break;
        }
    }

    if (optind < argc)
    {
        std::cerr << argv[0] << ": trailing args..\n";
        std::exit(1);
    }

    return opts;
}

}

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;

    rclcpp::init(argc, argv, rclcpp::InitOptions());

    auto opts = parseOpts(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("simple_server");

    RCLCPP_INFO(node->get_logger(), "spin period (ms): %zu", opts.spinPeriodMilliseconds);

    auto gridPub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("simple_server/occupancy_grid", 1);

    auto odomPub = node->create_publisher<nav_msgs::msg::Odometry>("simple_server/odometry", 1);

    auto executor = rclcpp::executors::SingleThreadedExecutor{ };

    executor.add_node(node);

    auto spinPeriod = std::chrono::milliseconds(opts.spinPeriodMilliseconds);

    while (rclcpp::ok())
    {
        const auto rateDeadline = std::chrono::steady_clock::now() + spinPeriod;

        auto gridMsg = initializeOccupancyGrid();

        if (gridMsg)
            gridPub->publish(std::move(gridMsg));

        auto odomMsg = initializeOdometry();

        if (odomMsg)
            odomPub->publish(std::move(odomMsg));

        std::this_thread::sleep_until(rateDeadline);
    }

    return 0;
}
