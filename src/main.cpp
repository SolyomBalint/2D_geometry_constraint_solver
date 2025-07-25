// General STD/STL headers
#include <cstdlib>
#include <exception>
#include <format>

// Custom headers
#include "./utils/graph.hpp"
#include "./constraint_solver/constraint_equation_solver.hpp"

// Thirdparty headers not needed for rendering
#include <argparse/argparse.hpp>
#include <iostream>
#include <memory>
#include <ostream>
#include <span>
#include <spdlog/common.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

namespace {
// NOLINTNEXTLINE note this should be investigated
const auto MAIN_LOGGER = spdlog::stdout_color_mt("MAIN");

int convertLogLevel(const std::string& input)
{
    if (input == "TRACE") {
        return spdlog::level::trace;
    }
    if (input == "DEBUG") {
        return spdlog::level::debug;
    }
    if (input == "INFO") {
        return spdlog::level::info;
    }
    if (input == "WARN") {
        return spdlog::level::warn;
    }
    if (input == "ERROR") {
        return spdlog::level::err;
    }
    if (input == "CRITICAL") {
        return spdlog::level::critical;
    }

    return -1;
}

void initParserArgumnets(argparse::ArgumentParser& argparser)
{
    argparser.add_argument("--log-level")
        .help("Define the log-level of the program. Defuaults to INFO."
              "Allowed values: [TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL]")
        .default_value("INFO");
    argparser.add_argument("--gui")
        .help("Whether to enable GUI when running the program, defaults to false")
        .default_value(false);
}

void parseArguments(argparse::ArgumentParser& argparser, std::span<char*> args)
{
    try {
        // NOLINTNEXTLINE(cppcoreguidelines-narrowing-conversions, bugprone-narrowing-conversions)
        argparser.parse_args(args.size(), args.data());
    } catch (const std::exception& err) {
        std::cerr << err.what() << '\n';
        // NOLINTNEXTLINE(concurrency-mt-unsafe)
        std::exit(-1);
    }

    {
        auto inputLogLevel = argparser.get<std::string>("--log-level");
        auto logLevel = convertLogLevel(inputLogLevel);

        if (logLevel == -1) {
            std::cerr << std::format("Invalid log level input: {}, "
                                     "Allowed values: [TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL]",
                inputLogLevel)
                      << '\n';
            // NOLINTNEXTLINE(concurrency-mt-unsafe)
            std::exit(-1);
        }

        // Sets GLOBAL log level
        spdlog::set_level(static_cast<spdlog::level::level_enum>(logLevel));
    }
}
} // namespace

int main(int argc, char* argv[])
{
    argparse::ArgumentParser argparser("2D Geometry Constraint Solver", "0.0.0");
    initParserArgumnets(argparser);
    parseArguments(argparser, std::span<char*> { argv, static_cast<size_t>(argc) });

    MAIN_LOGGER->info("Graph tests beginning");
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
    std::cout << "Cut vertices:" << '\n';

    for (auto& cutVertice : cutVertices) {
        std::cout << cutVertice.tempName << '\n';
    }

    auto out = Solver::calculatePointToPointDistanceTriangle(8, 8, 8);

    auto gui = argparser.get<bool>("--gui");

    if (gui) { }

    return 0;
}
