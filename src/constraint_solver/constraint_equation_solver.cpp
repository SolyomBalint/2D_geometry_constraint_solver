#include "constraint_equation_solver.hpp"

#include <autodiff/forward/real.hpp>
#include <autodiff/forward/dual.hpp>
#include <autodiff/forward/real/eigen.hpp>
#include <format>
#include <Eigen/Dense>
using namespace autodiff;
using namespace Eigen;

VectorX<dual> f(const VectorX<dual>& x)
{
    dual t1 = x(0);
    dual t2 = x(1);
    dual r = sqrt(13.0);

    VectorX<dual> res(2);
    res(0) = r * cos(t1) - t2;
    res(1) = r * sin(t1) - (t2 * t2 - 1.0);
    return res;
}

dual g(dual x, dual y) { return pow(x, 2) + pow(y, 2) - 13; }
dual h(dual x, dual y) { return pow(x, 2) - y - 1; }

void testJacobian()
{
    double error = 0.1;
    // TODO: Create dynamic vectors from these so that this code is actually reusable not just a POC
    double previousX { 0.0 };
    double previousY { 0.0 };

    dual x = 1;
    dual y = 1;

    while (true) {
        Eigen::Matrix2d jacobian { { derivative(g, wrt(x), at(x, y)), derivative(g, wrt(y), at(x, y)) },
            { derivative(h, wrt(x), at(x, y)), derivative(h, wrt(y), at(x, y)) } };

        Eigen::Vector2d rightside { -(g(x, y).val), -(h(x, y).val) };
        Eigen::Vector2d s = jacobian.colPivHouseholderQr().solve(rightside);

        std::cout << "#############################Debug prints#############################\n";

        std::cout << std::format("n is x={},y={}", x.val, y.val) << std::endl;
        std::cout << std::format("n+1 is x={},y={}", s[0], s[1]) << std::endl;

        if (abs(previousX - x.val) < error && abs(previousY - y.val) < error) {
            std::cout << std::format("The solution is x={},y={}", x.val, y.val) << std::endl;
            break;
        }

        previousX = x.val;
        previousY = y.val;
        x += s[0];
        y += s[1];
    }

    // Vector2d x = { 1.0, 2.0 }; // Initial guess: t1, t2
    //
    // const int max_iter = 100;
    // const double eps_f = 1e-8;
    // const double eps_dx = 1e-8;
    //
    // std::cout << "Starting Newton-Raphson iterations...\n";
    //
    // for (int i = 0; i < max_iter; ++i) {
    //     // Cast to autodiff type for Jacobian calculation
    //     VectorX<dual> x_dual = x.cast<dual>();
    //
    //     // Evaluate function f(x) and compute Jacobian J(x)
    //     VectorX<dual> fx = f(x_dual);
    //     MatrixXd J = jacobian(f, wrt(x_dual), at(x_dual), fx);
    //
    //     // Solve for Newton step: J * delta = f(x)
    //     VectorXd delta = J.colPivHouseholderQr().solve(fx.cast<double>());
    //
    //     // Update the estimate
    //     x = x - delta;
    //
    //     // Output iteration info
    //     std::cout << "Iter " << i << ": x = [" << x(0) << ", " << x(1) << "]"
    //               << ", |f| = " << fx.norm() << ", |dx| = " << delta.norm() << std::endl;
    //
    //     // Check stopping criteria
    //     if (fx.norm() < eps_f || delta.norm() < eps_dx) {
    //         std::cout << "Converged.\n";
    //         break;
    //     }
    // }
    //
    // std::cout << "\nFinal parameters:\n";
    // std::cout << "t1 = " << x(0) << ", t2 = " << x(1) << "\n";
    //
    // // Evaluate the actual intersection points
    // double r = std::sqrt(13.0);
    // Vector2d p1 { r * std::cos(x(0)), r * std::sin(x(0)) };
    // Vector2d p2 { x(1), x(1) * x(1) - 1.0 };
    //
    // std::cout << "\nIntersection point (should be the same for both):\n";
    // std::cout << "r1(t1) = [" << p1(0) << ", " << p1(1) << "]\n";
    // std::cout << "r2(t2) = [" << p2(0) << ", " << p2(1) << "]\n";
}
