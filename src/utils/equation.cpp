#include "equation.hpp"

namespace mathutils {
void Constant::print(std::ostream& os) const
{
    if (!exponent) {
        os << constValue;
        return;
    }

    os << constValue << '^' << *exponent;
}

double Constant::evaluate(const std::unordered_map<common::Uuid, double>& variableValueMapping) const
{
    if (!exponent) {
        return constValue;
    }

    return std::pow(constValue, exponent->evaluate(variableValueMapping));
}

// This is code duplication, if it gets bigger then minimal functions create parent class for Constant and Variable
void Variable::print(std::ostream& os) const
{
    if (!exponent) {
        os << variableName;
        return;
    }

    os << variableName << '^' << *exponent;
}

double Variable::evaluate(const std::unordered_map<common::Uuid, double>& variableValueMapping) const
{

    auto iter = variableValueMapping.find(getUuid());
    if (iter == variableValueMapping.end()) {
        throw std::invalid_argument(std::format("Missing value for variable {} at evaluation", variableName));
    }

    if (!exponent) {
        return iter->second;
    }

    return std::pow(iter->second, exponent->evaluate(variableValueMapping));
}
} // namespace mathutils
