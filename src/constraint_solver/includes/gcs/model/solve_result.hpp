#ifndef GCS_MODEL_SOLVE_RESULT_HPP
#define GCS_MODEL_SOLVE_RESULT_HPP

// General STD/STL headers
#include <string>
#include <utility>

// Custom headers
#include <gcs/export.hpp>

namespace Gcs {

/// @brief Outcome status of a subproblem solve attempt.
enum class SolveStatus {
    Success, ///< Component solved, element positions updated.
    Unsupported, ///< No solver recognized this configuration.
    Failed ///< Solver matched but numerical solving failed.
};

/// @brief Result of a subproblem solver invocation.
struct GCS_API SolveResult {
    SolveStatus status;
    std::string message;

    /**
     * @brief Create a successful result.
     * @return SolveResult with @c SolveStatus::Success.
     */
    static SolveResult success()
    {
        return { .status = SolveStatus::Success, .message = {} };
    }

    /**
     * @brief Create an unsupported-configuration result.
     * @param msg Description of why no solver matched.
     * @return SolveResult with @c SolveStatus::Unsupported.
     */
    static SolveResult unsupported(std::string msg)
    {
        return { .status = SolveStatus::Unsupported,
            .message = std::move(msg) };
    }

    /**
     * @brief Create a numerical-failure result.
     * @param msg Description of what went wrong during solving.
     * @return SolveResult with @c SolveStatus::Failed.
     */
    static SolveResult failed(std::string msg)
    {
        return { .status = SolveStatus::Failed, .message = std::move(msg) };
    }
};

} // namespace Gcs

#endif // GCS_MODEL_SOLVE_RESULT_HPP
