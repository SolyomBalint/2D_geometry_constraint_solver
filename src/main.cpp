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
#include "./constraint_solver/constraint_equation_solver.hpp"

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

    std::cout << "===========================" << std::endl;

    // Create a fresh constraint graph for testing defaultDecompositorFunc
    MathUtils::Graph<Gcs::Element, Gcs::Constraint> testGraph;
    auto testNode0 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 0, 0 }));
    auto testNode1 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 5, 0 }));
    auto testNode2 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 0, 5 }));
    auto testNode3 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 5, 5 }));
    auto testNode4 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 5, -5 }));
    auto testNode5 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { 0, -5 }));
    auto testNode6 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { -5, -5 }));
    auto testNode7 = testGraph.addNode(
        std::make_shared<Gcs::Element>(Gcs::Point { -5, 0 }));

    double squareSide = 6;
    auto testEdge0 = testGraph.addEdge(testNode0, testNode1,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge1 = testGraph.addEdge(testNode0, testNode2,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge2 = testGraph.addEdge(testNode0, testNode3,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge3 = testGraph.addEdge(testNode0, testNode4,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge4 = testGraph.addEdge(testNode0, testNode5,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge5 = testGraph.addEdge(testNode0, testNode6,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge6 = testGraph.addEdge(testNode0, testNode7,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));

    auto testEdge7 = testGraph.addEdge(testNode1, testNode2,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge8 = testGraph.addEdge(testNode2, testNode3,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge9 = testGraph.addEdge(testNode3, testNode4,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge10 = testGraph.addEdge(testNode4, testNode5,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge11 = testGraph.addEdge(testNode5, testNode6,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));
    auto testEdge12 = testGraph.addEdge(testNode6, testNode7,
        std::make_shared<Gcs::Constraint>(Gcs::DistanceConstraint(squareSide)));

    Gcs::GeometricConstraintSystem gcs {};
    gcs.solveGcsViaPipeline(testGraph);
    auto results = testGraph.getNodes();
    for (const auto& node : results) {
        std::cout << node.getStoredObj()->toString() << std::endl;
    }

    std::cout << "===========================" << std::endl;

    if (gui) { }

    return 0;
}
