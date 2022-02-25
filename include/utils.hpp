#ifndef MAP_TO_GRAPH_UTILS_HPP
#define MAP_TO_GRAPH_UTILS_HPP

#include <tuple>
#include <vector>

using hom_t = std::tuple<double, double, double>;

struct battery_model_t {
  double cell_capacity;
  int number_of_cells;

  // Coefficients for the equation https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9691840&tag=1 (21)
  // k = C_eff / C = d_0 + d1 * P_cell + d2 * P_cell^2 + d3 * P_cell^3
  double d0;
  double d1;
  double d2;
  double d3;
};


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

#endif
