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
point_t rotate_point(point_t p, double angle);

point_t segment_segment_intersection(const segment_t &s1, const segment_t &s2);

double segment_length(const segment_t &segment);

bool segments_intersect(const segment_t &s1, const segment_t &s2);

double angle_between_vectors(point_t p1, point_t p2, point_t p3);

double distance_between_points(point_t p1, point_t p2);

point_t gps_coordinates_to_meters(point_t p);

point_t meters_to_gps_coordinates(point_t p);

int generate_random_number();

bool polygon_convex(std::vector<point_t> polygon);

/*!
 * Get the rotation of the segment (as vector starting at first point) according to the Ox axis
 * @param segment Segment, rotation for which will be found
 * @return Angle of the rotation in radians
 */
double get_segment_rotation(segment_t segment);

/*!
 * Check if two numbers are close enough
 * @param n1 Number 1
 * @param n2 Number 2
 * @param eps Possible error
 * @return true if the distance between n1 and n2 is less than EPS
 */
template<typename T>
bool isclose(T n1, T n2, T eps=1e-10) {
    return std::abs(n1 - n2) < eps;
}

#endif

