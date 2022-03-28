//
// Created by mrs on 23.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP
#define THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP

#include <map>
#include <vector>
#include "MapPolygon.hpp"

struct shortest_path_calculation_error: public std::runtime_error {
    using runtime_error::runtime_error;
};

/*!
 * Class for calculation of the shortest path between point inside a polygon
 */
class ShortestPathCalculator {
private:
    std::vector<segment_t> m_polygon_segments;
    std::map<point_t, int> m_point_index;
    std::vector<point_t> m_polygon_points;
    std::vector<std::vector<double>> m_floyd_warshall_d;
    std::vector<std::vector<size_t>> m_next_vertex_in_path;

    void run_floyd_warshall();

    std::vector<int> polygon_nodes_seen_from_point(point_t point);

    std::vector<point_t> shortest_path_between_polygon_nodes(size_t i, size_t j);

    /*!
     *  Find the closest point in polygon to yhe given point
     * @param p point to which the closest point will be found
     * @return The closest point to p
     */
    point_t closest_polygon_point(point_t p);

public:
    /*!
     * Main and the only constructor of the calculator.
     * @param polygon polygon, bounds of which will define the shortest path
     */
    explicit ShortestPathCalculator(const MapPolygon &polygon);

    ShortestPathCalculator() = delete;

    /*!
     * Method for getting the approximate shortest path between two points
     * @note It works fine only all two points are located close to nodes of the map polygon.
     * @note The path can even be unfeasible (going through a no-fly zone)
     * @param p1 Point to find path from
     * @param p2 Point to find path to
     * @return Path between teo points, where path[0] = p1, path[-1] = p2
     */
    std::vector<point_t> get_approximate_shortest_path(point_t p1, point_t p2);

    std::vector<point_t> shortest_path_between_points(point_t p1, point_t p2);

};


#endif //THESIS_TRAJECTORY_GENERATOR_SHORTESTPATHCALCULATOR_HPP
