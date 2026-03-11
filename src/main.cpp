// General STD/STL headers
#include <cstdlib>
#include <iostream>

// Thirdparty headers
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <gcs/model/constraints.hpp>
#include <gcs/model/elements.hpp>
#include <gcs/model/gcs_data_structures.hpp>
#include <gcs/model/solve_result.hpp>
#include <gcs/orchestration/geometric_constraint_system.hpp>

#include <solving/component_solver.hpp>

namespace {
// NOLINTNEXTLINE note this should be investigated
const auto MAIN_LOGGER = spdlog::stdout_color_mt("MAIN");
} // namespace

int main()
{
    spdlog::set_level(spdlog::level::debug);
    MAIN_LOGGER->info("2D Geometry Constraint Solver starting...");

    using namespace Gcs;

    return 0;
}
