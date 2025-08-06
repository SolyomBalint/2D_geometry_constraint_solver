#ifndef COMMON_UUID_HPP
#define COMMON_UUID_HPP

#include <algorithm>
#include <array>
#include <functional>
#include <iterator>
#include <random>
#include <uuid.h>

namespace Common {

using Uuid = uuids::uuid;

/**
 * @brief Create a UUID based on the Mt19937 algorithm.
 *
 * @note the algorithm is seeded once per program run with std::random_device
 *
 * @return the generated UUID
 */
Uuid generateUuidMt19937();
} // namespace common::uuid

#endif // COMMON_UUID_HPP
