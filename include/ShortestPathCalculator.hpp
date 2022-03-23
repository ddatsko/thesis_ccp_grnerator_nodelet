//
// Created by mrs on 23.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP
#define THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP

#include <map>
#include <vector>
#include "MapPolygon.hpp"

/*!
 * Class for calculation of the shortest path between point inside a polygon
 */
class ShortestPathCalculator {
private:
    std::map<std::pair<double, double>, int> m_point_index;
    std::vector<std::vector<int>> m_distances;

public:
    /*!
     * Main and the only constructor of the calculator.
     * @param polygon polygon, bounds of which will define the shortest path
     */
    explicit ShortestPathCalculator(const MapPolygon &polygon);

    ShortestPathCalculator() = delete;

    std::vector<point_t> shortest_path_between_points(point_t p1, point_t p2);

};


#endif //THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP
