#ifndef MAP_TO_GRAPH_UTILS_HPP
#define MAP_TO_GRAPH_UTILS_HPP

#include <tuple>
#include <vector>
#include "custom_types.hpp"

using hom_t = std::tuple<double, double, double>;

hom_t cross_product(const hom_t &a, const hom_t &b);

/*!
 * Get the point of intersection of a segment and lone
 * @param p1 segment point 1
 * @param p2 segment point 2
 * @param line homography representation of the line
 * @return point of the intersection of the segment and the line
 */
std::pair<double, double> segment_line_intersection(const std::pair<double, double> &p1, 
                                                    const std::pair<double, double> &p2,
                                                    const hom_t &line);


/*!
 * Get the point, obtained by rotating the point p counter clockwise by angle
 * @param angle Angle of the counter clockwise point rotation
 * @param p Point to be rotated in the representation {x, y}
 * @return Points obrained by rotation
 */
std::pair<double, double> rotate_point(std::pair<double, double> p, double angle);

std::pair<double, double> segment_segment_intersection(const segment_t &s1, const segment_t &s2);

/*!
 * Check if two numbers are close enough
 * @param n1 Number 1
 * @param n2 Number 2
 * @param eps Possible error
 * @return true if the distance between n1 and n2 is less than eps
 */
template<typename T>
bool isclose(T n1, T n2, T eps=1e-10) {
    return std::abs(n1 - n2) < eps;
}

#endif

