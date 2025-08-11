// General STD/STL headers
#include <cstdlib>
#include <exception>
#include <format>

// Custom headers
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

// #include "./utils/graphs/graph_impls/default.hpp"
#include "./utils/graphs/graph_impls/ogdf_wrapper.hpp"

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/extended_graph_alg.h>

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

    auto out = Solver::calculatePointToPointDistanceTriangle(8, 8, 8);

    MathUtils::Graph<int, int> testGraph {};

    auto node1 = testGraph.addNode(std::make_shared<int>(1));
    auto node2 = testGraph.addNode(std::make_shared<int>(2));
    auto node3 = testGraph.addNode(std::make_shared<int>(3));
    auto node4 = testGraph.addNode(std::make_shared<int>(4));
    auto node5 = testGraph.addNode(std::make_shared<int>(5));

    auto edge1 = testGraph.addEdge(node1, node2, std::make_shared<int>(1));
    auto edge2 = testGraph.addEdge(node1, node3, std::make_shared<int>(1));

    auto edge3 = testGraph.addEdge(node3, node2, std::make_shared<int>(1));

    auto edge4 = testGraph.addEdge(node3, node4, std::make_shared<int>(1));
    auto edge5 = testGraph.addEdge(node3, node5, std::make_shared<int>(1));
    auto edge6 = testGraph.addEdge(node4, node5, std::make_shared<int>(1));

    auto outNodes = testGraph.getCutVertices();

    std::cout << outNodes.size() << std::endl;
    std::cout << *(outNodes[0].getStoredObj().get()) << std::endl;

    MathUtils::Graph<int, int> testGraphSeparationPairs {};
    auto node21 = testGraphSeparationPairs.addNode(std::make_shared<int>(1));
    auto node22 = testGraphSeparationPairs.addNode(std::make_shared<int>(2));
    auto node23 = testGraphSeparationPairs.addNode(std::make_shared<int>(3));
    auto node24 = testGraphSeparationPairs.addNode(std::make_shared<int>(4));

    auto edge21 = testGraphSeparationPairs.addEdge(node21, node22, std::make_shared<int>(8));
    auto edge22 = testGraphSeparationPairs.addEdge(node21, node23, std::make_shared<int>(9));
    auto edge23 = testGraphSeparationPairs.addEdge(node22, node23, std::make_shared<int>(10));
    auto edge24 = testGraphSeparationPairs.addEdge(node22, node24, std::make_shared<int>(11));
    auto edge25 = testGraphSeparationPairs.addEdge(node23, node24, std::make_shared<int>(12));

    MathUtils::Graph<int, int>::NodeType outNode1; // Actual objects, not pointers
    MathUtils::Graph<int, int>::NodeType outNode2;

    testGraphSeparationPairs.getSeparationPairs(&outNode1, &outNode2); // Pass addresses

    testGraphSeparationPairs.separateByVerticesByDuplication({ outNode1, outNode2 });

    ogdf::Graph test {};

    auto nodex = test.newNode(5);
    auto nodey = test.newNode(6);
    auto nodez = test.newNode(7);
    test.newEdge(nodex, nodey);

    ogdf::List<ogdf::node> testList {};
    testList.pushBack(nodex);
    testList.pushBack(nodey);
    testList.search(nodex);

    ogdf::Graph subGraph {};
    ogdf::ListIterator<ogdf::node> it { testList.begin() };
    ogdf::inducedSubGraph(test, it, subGraph);

    // std::cout << test.edges.size() << std::endl;
    std::cout << test.nodes.head()->index() << std::endl;

    // std::cout << subGraph.edges.size() << std::endl;
    std::cout << subGraph.nodes.head()->index() << std::endl;

    //
    // ogdf::Graph test2 {};
    // test2.newNode(test.nodes.tail()->index());
    //
    // test2.delNode(test.nodes.tail());
    //
    // std::cout << test.nodes.tail()->index() << "\n";
    // std::cout << test2.nodes.tail()->index() << "\n";

    // this fails
    // test.delNode(test2.nodes.tail());

    // ogdf::List<ogdf::node> nodeTest {};
    // test.allNodes(nodeTest);
    //
    // std::cout << "\n";
    //
    // std::cout << nodeTest.size() << "\n";
    // std::cout << test.nodes.size() << "\n";
    //
    // std::cout << "\n";
    //
    // nodeTest.clear();
    //
    // std::cout << nodeTest.size() << "\n";
    // std::cout << test.nodes.size() << "\n";

    auto gui = argparser.get<bool>("--gui");

    if (gui) { }

    return 0;
}
