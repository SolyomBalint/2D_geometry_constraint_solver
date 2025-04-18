// General STD/STL headers
#include <format>

// Custom headers
#include "./utils/graph.hpp"

// Thirdparty headers not needed for rendering
#include <argparse/argparse.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

auto mainLogger = spdlog::stdout_color_mt("MAIN");

int ConvertLogLevel(const std::string& input)
{
    if (input == "TRACE") {
        return spdlog::level::trace;
    } else if (input == "DEBUG") {
        return spdlog::level::debug;
    } else if (input == "INFO") {
        return spdlog::level::info;
    } else if (input == "WARN") {
        return spdlog::level::warn;
    } else if (input == "ERROR") {
        return spdlog::level::err;
    } else if (input == "CRITICAL") {
        return spdlog::level::critical;
    } else {
        return -1;
    }
}

void InitParserArgumnets(argparse::ArgumentParser& argparser)
{
    argparser.add_argument("--log-level")
        .help("Define the log-level of the program. Defuaults to INFO."
              "Allowed values: [TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL]")
        .default_value("INFO");
    argparser.add_argument("--gui")
        .help("Whether to enable GUI when running the program, defaults to false")
        .default_value(false);
}

void ParseArguments(argparse::ArgumentParser& argparser, int argc, char* argv[])
{
    try {
        argparser.parse_args(argc, argv);
    } catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::exit(-1);
    }

    {
        auto inputLogLevel = argparser.get<std::string>("--log-level");
        auto logLevel = ConvertLogLevel(inputLogLevel);

        if (logLevel == -1) {
            std::cerr << std::format("Invalid log level input: {}, "
                                     "Allowed values: [TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL]",
                inputLogLevel)
                      << std::endl;
            std::exit(-1);
        }

        // Sets GLOBAL log level
        spdlog::set_level(static_cast<spdlog::level::level_enum>(logLevel));
    }
}

int main(int argc, char* argv[])
{
    argparse::ArgumentParser argparser("2D Geometry Constraint Solver", "0.0.0");
    InitParserArgumnets(argparser);
    ParseArguments(argparser, argc, argv);

    MathUtils::Node node1;
    MathUtils::Node node2;
    MathUtils::Node node3;
    MathUtils::Node node4;
    MathUtils::Node node5;

    node1.m_edges.emplace_back(node2);
    node1.m_edges.emplace_back(node3);

    node2.m_edges.emplace_back(node1);
    node2.m_edges.emplace_back(node3);

    node3.m_edges.emplace_back(node1);
    node3.m_edges.emplace_back(node2);
    node3.m_edges.emplace_back(node4);
    node3.m_edges.emplace_back(node5);

    node4.m_edges.emplace_back(node5);
    node4.m_edges.emplace_back(node3);

    node5.m_edges.emplace_back(node4);
    node5.m_edges.emplace_back(node3);

    std::vector<MathUtils::Node> temp = { node1, node2, node3, node4, node5 };

    auto testGraph = std::make_unique<MathUtils::UndirectedGraph>(temp);

    auto cutVertices = testGraph->getCutVertices();
    std::cout << "Cut vertices:" << std::endl;

    for (auto& cutVertice : cutVertices)
        std::cout << cutVertice.tempName << std::endl;

    auto gui = argparser.get<bool>("--gui");
    if (gui) { }

    return 0;
}
