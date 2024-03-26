#include <getopt.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

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

int main(int argc, char **argv)
{
    using namespace std::chrono_literals;

    rclcpp::init(argc, argv, rclcpp::InitOptions());

    auto opts = parseOpts(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("simple_server"), "spin period (ms): %zu", opts.spinPeriodMilliseconds);

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

    auto spinPeriod = std::chrono::milliseconds(opts.spinPeriodMilliseconds);

    while (rclcpp::ok())
    {
        const auto rateDeadline = std::chrono::steady_clock::now() + spinPeriod;

        RCLCPP_INFO(rclcpp::get_logger("simple_server"), "before spin");

        executor.spin_all(spinPeriod);

        RCLCPP_INFO(rclcpp::get_logger("simple_server"), "after spin");

        std::this_thread::sleep_until(rateDeadline);
    }

    return 0;
}
