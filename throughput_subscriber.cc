#include <getopt.h>
#include <spdlog/spdlog.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>

namespace {

struct Options
{
    size_t window{1000000};
};

void showHelp(const std::string &name)
{
    std::cout << "usage: " << name << " OPTIONS\n"
        "OPTIONS:\n"
        " -w | --window                       window for measuring bandwidth.\n"
        " -h | --help                         show this help.\n";
}

Options parseOpts(int argc, char **argv)
{
    static constexpr auto shortOpts = "w:h";
    static constexpr struct option longOpts[] = {
        { "window", required_argument, nullptr, 'w' },
        { "help", no_argument, nullptr, 'h' },
        { nullptr, 0, nullptr, 0 }
    };

    Options opts{ };

    for (int c = 0; (c = getopt_long(argc, argv, shortOpts, longOpts, nullptr)) >= 0; )
    {
        switch (c)
        {
            case 'w':
                opts.window = std::clamp<size_t>(std::stod(optarg), 1, 1e9);
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

struct Stats
{
    size_t count{ };
    size_t finishedRounds{ };
    std::chrono::time_point<std::chrono::steady_clock> start{ };
    std::chrono::time_point<std::chrono::steady_clock> firstStart{ };
    bool started{ };
};

}

int main(int argc, char **argv)
{
    using Clock = std::chrono::steady_clock;

    using microseconds = std::chrono::microseconds;

    rclcpp::init(argc, argv, rclcpp::InitOptions());

    auto opts = parseOpts(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("throughput_subscriber");

    auto stats = Stats{ };

    auto payloadSub = node->create_subscription<std_msgs::msg::ByteMultiArray>(
        "payload",
        1,
        [&stats, &opts](const std_msgs::msg::ByteMultiArray &payload)
        {
            // styled after z_thr_sub.c example
            if (stats.count == 0) {
                stats.start = Clock::now();
                if (!stats.started)
                {
                    stats.firstStart = stats.start;
                    stats.started = true;
                }
                stats.count++;
            }
            else if (stats.count < opts.window)
            {
                stats.count++;
            }
            else
            {
                stats.finishedRounds++;

                auto now = Clock::now();

                auto elapsedMicroseconds = std::chrono::duration_cast<microseconds>(now - stats.start).count();

                auto elapsedSeconds = elapsedMicroseconds / 1e6;

                auto msgsPerSecond = static_cast<double>(opts.window) / elapsedSeconds;

                auto bytesPerSecond = static_cast<double>(payload.data.size() * opts.window) / elapsedSeconds;

                spdlog::info("{} msg/s, {} MB/s"
                             , msgsPerSecond
                             , bytesPerSecond * 1e-6);

                stats.start = Clock::now();

                stats.count = 0;
            }
        });

    rclcpp::spin(node);

    return 0;
}
