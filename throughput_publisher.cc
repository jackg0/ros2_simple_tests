#include <getopt.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

namespace {

struct Options
{
    size_t payloadSize{1};
};

void showHelp(const std::string &name)
{
    std::cout << "usage: " << name << " OPTIONS\n"
        "OPTIONS:\n"
        " -p | --payload-size     payload size in bytes; valid bounds [1, 1e9].\n"
        " -h | --help             show this help.\n";
}

Options parseOpts(int argc, char **argv)
{
    static constexpr auto shortOpts = "p:h";
    static constexpr struct option longOpts[] = {
        { "payload-size", required_argument, nullptr, 'p' },
        { "help", no_argument, nullptr, 'h' },
        { nullptr, 0, nullptr, 0 }
    };

    Options opts{ };

    for (int c = 0; (c = getopt_long(argc, argv, shortOpts, longOpts, nullptr)) >= 0; )
    {
        switch (c)
        {
            case 'p':
                opts.payloadSize = std::clamp<size_t>(std::stod(optarg), 1, 1e9);
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
    rclcpp::init(argc, argv, rclcpp::InitOptions());

    auto opts = parseOpts(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("throughput_publisher");

    auto payloadPub = node->create_publisher<std_msgs::msg::ByteMultiArray>("payload", 1);

    auto payload = std_msgs::msg::ByteMultiArray{ };

    payload.data.resize(opts.payloadSize);

    while (rclcpp::ok())
    {
        payloadPub->publish(payload);
    }

    return 0;
}
