#include "equation.hpp"

namespace mathutils {
std::ostream& operator<<(std::ostream& os, mathutils::Expression const& e)
{
    e.print(os);

    if (e.exponent) {
        os << '^' << *(e.exponent);
    }

    return os;
}

double Constant::evaluate(const std::unordered_map<common::Uuid, double>& variableValueMapping) const
{
    if (!exponent) {
        return constValue;
    }

    return std::pow(constValue, exponent->evaluate(variableValueMapping));
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

double TwoOperandOperation::evaluate(const std::unordered_map<common::Uuid, double>& variableValueMapping) const
{
    auto result = do_operator(lhs->evaluate(variableValueMapping), rhs->evaluate(variableValueMapping));

    if (exponent) {
        return std::pow(result, exponent->evaluate(variableValueMapping));
    }

    return result;
}

} // namespace mathutils
