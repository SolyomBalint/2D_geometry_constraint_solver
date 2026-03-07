// General STD/STL headers
#include <cstdlib>
#include <iostream>

// Thirdparty headers
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "./constraint_solver/src/model/constraints.hpp"
#include "./constraint_solver/src/model/elements.hpp"
#include "./constraint_solver/src/model/gcs_data_structures.hpp"
#include "./constraint_solver/src/model/solve_result.hpp"
#include "./constraint_solver/src/orchestration/geometric_constraint_system.hpp"
#include "./constraint_solver/src/solving/component_solver.hpp"

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
