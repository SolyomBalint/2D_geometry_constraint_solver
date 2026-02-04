#ifndef EQUATION_PRIMITIVES_HPP
#define EQUATION_PRIMITIVES_HPP

// Thirdparty headers
#include <autodiff/forward/dual.hpp>

namespace Gcs::Equations {

using autodiff::dual;

/**
 * @brief Point-to-point squared-distance equation.
 *
 * Returns a lambda: @c (x - x0)^2 + (y - y0)^2 - d^2 = 0.
 * Equals zero when the unknown point @c (x, y) is exactly distance
 * @p d from the fixed point @c (@p x0, @p y0).
 *
 * @param x0 Fixed point X coordinate.
 * @param y0 Fixed point Y coordinate.
 * @param d Required distance.
 * @return Autodiff-compatible lambda @c (dual x, dual y) -> dual.
 */
inline auto pointToPointDistance(dual x0, dual y0, dual d)
{
    return [x0, y0, d](dual x, dual y) -> dual {
        return pow(x - x0, 2) + pow(y - y0, 2) - pow(d, 2);
    };
}

/**
 * @brief Collinearity equation — point lies on line through two points.
 *
 * Returns a lambda: @c (xb - xa)(y - ya) - (yb - ya)(x - xa) = 0.
 * This is the cross-product test; it equals zero when @c (x, y) is
 * collinear with @c (@p xa, @p ya) and @c (@p xb, @p yb).
 *
 * @param xa Line-defining point A, X coordinate.
 * @param ya Line-defining point A, Y coordinate.
 * @param xb Line-defining point B, X coordinate.
 * @param yb Line-defining point B, Y coordinate.
 * @return Autodiff-compatible lambda @c (dual x, dual y) -> dual.
 */
inline auto pointOnLine(dual xa, dual ya, dual xb, dual yb)
{
    return [xa, ya, xb, yb](dual x, dual y) -> dual {
        return (xb - xa) * (y - ya) - (yb - ya) * (x - xa);
    };
}

/**
 * @brief Signed perpendicular distance from a point to a line.
 *
 * Returns a lambda encoding the constraint that the perpendicular
 * distance from the unknown point @c (x, y) to the line through
 * @c (@p xa, @p ya) and @c (@p xb, @p yb) equals @p d.
 *
 * Uses the cross-product form:
 * @c (xb - xa)(y - ya) - (yb - ya)(x - xa) - d * |AB| = 0
 * where @c |AB| is the length of the line segment. This avoids
 * square roots in the equation (the length is a known constant).
 *
 * @param xa Line point A, X coordinate.
 * @param ya Line point A, Y coordinate.
 * @param xb Line point B, X coordinate.
 * @param yb Line point B, Y coordinate.
 * @param d Required signed distance.
 * @param lineLength Precomputed Euclidean length of segment AB.
 * @return Autodiff-compatible lambda @c (dual x, dual y) -> dual.
 */
inline auto pointToLineDistance(
    dual xa, dual ya, dual xb, dual yb, dual d, dual lineLength)
{
    return [xa, ya, xb, yb, d, lineLength](dual x, dual y) -> dual {
        return (xb - xa) * (y - ya) - (yb - ya) * (x - xa) - d * lineLength;
    };
}

/**
 * @brief Angle equation between two line direction vectors.
 *
 * Returns a lambda encoding the constraint that the angle between
 * direction vector @c (dx1, dy1) and direction vector @c (dx2, dy2)
 * satisfies: @c dot(d1, d2) - |d1||d2|cos(angle) = 0.
 *
 * The direction vectors and their lengths are precomputed constants
 * captured by the lambda; the unknowns @c (x, y) are not used
 * directly in this formulation. This primitive is intended to be
 * combined with other equations that define the line endpoints.
 *
 * @param dx1 First direction vector X component.
 * @param dy1 First direction vector Y component.
 * @param dx2 Second direction vector X component.
 * @param dy2 Second direction vector Y component.
 * @param cosAngle Cosine of the required angle.
 * @param len1 Length of first direction vector.
 * @param len2 Length of second direction vector.
 * @return Autodiff-compatible lambda @c (dual x, dual y) -> dual.
 */
inline auto lineToLineAngle(
    dual dx1, dual dy1, dual dx2, dual dy2, dual cosAngle, dual len1, dual len2)
{
    return [dx1, dy1, dx2, dy2, cosAngle, len1, len2](dual x, dual y) -> dual {
        // x and y are unused — this equation constrains precomputed
        // direction vectors. Solvers that use this primitive will
        // typically compose it with endpoint equations.
        (void)x;
        (void)y;
        return (dx1 * dx2 + dy1 * dy2) - len1 * len2 * cosAngle;
    };
}

} // namespace Gcs::Equations

#endif // EQUATION_PRIMITIVES_HPP
