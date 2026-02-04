// General STD/STL headers
#include <cstdlib>
#include <iostream>

// Thirdparty headers
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "./constraint_solver/src/component_solver.hpp"
#include "./constraint_solver/src/constraints.hpp"
#include "./constraint_solver/src/elements.hpp"
#include "./constraint_solver/src/gcs_data_structures.hpp"
#include "./constraint_solver/src/geometric_constraint_system.hpp"
#include "./constraint_solver/src/solve_result.hpp"

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
