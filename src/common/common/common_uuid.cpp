#include "common_uuid.hpp"

namespace {
class UuidGenerator {
    std::mt19937 generator_;
    uuids::uuid_random_generator gen_;

public:
    UuidGenerator()
        : generator_(std::random_device {}())
        , gen_(generator_)
    {
    }
    Common::Uuid generate() { return gen_(); }
};

static UuidGenerator generator {};
}

namespace Common {
Uuid generateUuidMt19937() { return generator.generate(); }
}
