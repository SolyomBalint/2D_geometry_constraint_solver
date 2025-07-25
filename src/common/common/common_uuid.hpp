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

inline Uuid generateUuidMt19937()
{
    std::random_device randomDevice;
    auto seedData = std::array<int, std::mt19937::state_size> {};
    std::generate(std::begin(seedData), std::end(seedData), std::ref(randomDevice));
    std::seed_seq seq(std::begin(seedData), std::end(seedData));
    std::mt19937 generator(seq);
    uuids::uuid_random_generator gen { generator };

    return gen();
}
} // namespace common::uuid

#endif // COMMON_UUID_HPP
