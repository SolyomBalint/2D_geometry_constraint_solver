// General STD/STL headers
#include <cstdlib>
#include <exception>
#include <format>

// Custom headers
// #include "./constraint_solver/constraint_equation_solver.hpp"

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

#include "./constraint_solver/geometric_constraint_system.hpp"

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
        .help(
            "Whether to enable GUI when running the program, defaults to false")
        .default_value(false);
}

void parseArguments(argparse::ArgumentParser& argparser, std::span<char*> args)
{
    try {
        // NOLINTNEXTLINE(cppcoreguidelines-narrowing-conversions,
        // bugprone-narrowing-conversions)
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
            std::cerr << std::format(
                "Invalid log level input: {}, "
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
    argparse::ArgumentParser argparser(
        "2D Geometry Constraint Solver", "0.0.0");
    initParserArgumnets(argparser);
    parseArguments(
        argparser, std::span<char*> { argv, static_cast<size_t>(argc) });

    auto gui = argparser.get<bool>("--gui");

    MathUtils::Graph<Gcs::Element, Gcs::Constraint> constraintGraph;
    auto node1 = constraintGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 0.0, 0.0 }));
    auto node2 = constraintGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 0.0, 5.0 }));
    auto node3 = constraintGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 5.0, 0.0 }));
    auto node4 = constraintGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 5.0, 5.0 }));

    auto edge1 = constraintGraph.addEdge(node1, node2,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));
    auto edge2 = constraintGraph.addEdge(node1, node3,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));
    auto edge3 = constraintGraph.addEdge(node2, node4,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));
    auto edge4 = constraintGraph.addEdge(node3, node4,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));
    auto edge5 = constraintGraph.addEdge(node2, node3,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));

    auto [sep1, sep2] = constraintGraph.getSeparationPairs();
    auto vec = constraintGraph.separateByVerticesByDuplication(
        { sep1.value(), sep2.value() });

    MathUtils::Graph<Gcs::Element, Gcs::Constraint> checkProblems { std::move(
        vec[0]) };
    checkProblems.getSeparationPairs();

    MathUtils::Graph<Gcs::Element, Gcs::Constraint> checkProblems2 { std::move(
        vec[1]) };
    checkProblems2.getSeparationPairs();

    std::cout << "===========================" << std::endl;

    // Create a fresh constraint graph for testing defaultDecompositorFunc
    MathUtils::Graph<Gcs::Element, Gcs::Constraint> testGraph;
    auto testNode1 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 0.0, 0.0 }));
    auto testNode2 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 0.0, 5.0 }));
    auto testNode3 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 5.0, 0.0 }));
    auto testNode4 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 5.0, 5.0 }));

    auto testEdge1 = testGraph.addEdge(testNode1, testNode2,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));
    auto testEdge2 = testGraph.addEdge(testNode1, testNode3,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));
    auto testEdge3 = testGraph.addEdge(testNode2, testNode4,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));
    auto testEdge4 = testGraph.addEdge(testNode3, testNode4,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));
    auto testEdge5 = testGraph.addEdge(testNode2, testNode3,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(5.0)));

    auto subGraphs = Gcs::defaultDecompositorFunc(testGraph);
    std::cout << subGraphs.size() << std::endl;

    if (gui) { }

    return 0;
}
