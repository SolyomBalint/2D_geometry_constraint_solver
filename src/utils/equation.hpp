#ifndef MATHUTILS_EQUATION_HPP
#define MATHUTILS_EQUATION_HPP

#include <common/common_uuid.hpp>
#include <ostream>
#include <uuid.h>

namespace mathutils {

class Expression {
public:
    virtual double evaluate(const std::unordered_map<common::Uuid, double>& variableValueMapping) const = 0;

    virtual ~Expression() = default;

    explicit Expression()
        : UUID(common::generateUuidMt19937())
    {
    }

    Expression(const Expression&) = delete;
    Expression& operator=(const Expression&) = delete;
    Expression(Expression&&) = default;
    Expression& operator=(Expression&&) = delete;

    const common::Uuid& getUuid() const { return UUID; }

private:
    const common::Uuid UUID;

    virtual void print(std::ostream& os) const = 0;

    friend std::ostream& operator<<(std::ostream& os, mathutils::Expression const& e);
};

inline std::ostream& operator<<(std::ostream& os, mathutils::Expression const& e)
{
    e.print(os);
    return os;
}

class Constant final : public Expression {
public:
    explicit Constant(double constValue, const std::shared_ptr<const Expression> exponent = nullptr)
        : Expression()
        , constValue(constValue)
        , exponent(exponent)
    {
    }

    virtual double evaluate(const std::unordered_map<common::Uuid, double>& variableValueMapping) const override;

private:
    const double constValue;
    const std::shared_ptr<const Expression> exponent;

    virtual void print(std::ostream& os) const override;
};

class Variable final : public Expression {
public:
    explicit Variable(std::string variableName = "", const std::shared_ptr<const Expression> exponent = nullptr)
        : Expression()
        , variableName(variableName)
        , exponent(exponent)
    {
    }

    virtual double evaluate(const std::unordered_map<common::Uuid, double>& variableValueMapping) const override;

private:
    const std::string variableName;

    // Expressions are designed to be immutable, so using this approach should be safe, and save a little bit of memory
    const std::shared_ptr<const Expression> exponent;

    virtual void print(std::ostream& os) const override;
};

class TwoOperandOperation : public Expression {
public:
    explicit TwoOperandOperation(std::unique_ptr<const Expression> lhs, std::unique_ptr<const Expression> rhs)
        : Expression()
        , lhs(std::move(lhs))
        , rhs(std::move(rhs))
    {
    }

    virtual double evaluate(const std::unordered_map<common::Uuid, double>& variableValueMapping) const override
    {
        return do_operator(lhs->evaluate(variableValueMapping), rhs->evaluate(variableValueMapping));
    }

private:
    const std::unique_ptr<const Expression> lhs;
    const std::unique_ptr<const Expression> rhs;

    virtual char get_operator() const = 0;

    virtual double do_operator(double lhs, double rhs) const = 0;

    virtual void print(std::ostream& os) const override { os << '(' << *lhs << get_operator() << *rhs << ')'; }
};

class Addition final : public TwoOperandOperation {
public:
    using TwoOperandOperation::TwoOperandOperation;

private:
    virtual char get_operator() const override { return '+'; }

    virtual double do_operator(double lhs, double rhs) const override { return lhs + rhs; }
};

class Subtraction final : public TwoOperandOperation {
public:
    using TwoOperandOperation::TwoOperandOperation;

private:
    virtual char get_operator() const override { return '-'; }

    virtual double do_operator(double lhs, double rhs) const override { return lhs - rhs; }
};

class Multiplication final : public TwoOperandOperation {
public:
    using TwoOperandOperation::TwoOperandOperation;

private:
    virtual char get_operator() const override { return '*'; }

    virtual double do_operator(double lhs, double rhs) const override { return lhs * rhs; }
};

class Division final : public TwoOperandOperation {
public:
    using TwoOperandOperation::TwoOperandOperation;

private:
    virtual char get_operator() const override { return '/'; }

    virtual double do_operator(double lhs, double rhs) const override { return lhs / rhs; }
};

class Equation { };

} // namespace mathutils

#endif // MATHUTILS_EQUATION_HPP
