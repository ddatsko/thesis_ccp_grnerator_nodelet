#include "utils.hpp"
#include <vector>
#include <cmath>
#include <iostream>


hom_t cross_product(const hom_t &a, const hom_t &b) {
    return {std::get<1>(a) * std::get<2>(b) - std::get<2>(a) * std::get<1>(b),
            std::get<2>(a) * std::get<0>(b) - std::get<0>(a) * std::get<2>(b),
            std::get<0>(a) * std::get<1>(b) - std::get<1>(a) * std::get<0>(b)};
}

std::pair<double, double> segment_line_intersection(const std::pair<double, double> &p1,
                                                    const std::pair<double, double> &p2,
                                                    const hom_t &line) {
    hom_t p1_h = {p1.first, p1.second, 1};
    hom_t p2_h = {p2.first, p2.second, 1};
    hom_t segment_line = cross_product(p1_h, p2_h);
    hom_t p_intersection_h = cross_product(segment_line, line);
    return {std::get<0>(p_intersection_h) / std::get<2>(p_intersection_h),
            std::get<1>(p_intersection_h) / std::get<2>(p_intersection_h)};
}

std::pair<double, double> rotate_point(std::pair<double, double> p, double angle) {
  return {p.first * std::cos(angle) - p.second * std::sin(angle), p.second * std::cos(angle) + p.first * std::sin(angle)};
}