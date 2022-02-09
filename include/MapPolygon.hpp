#ifndef MAP_TO_GRAPH_MAPPOLYGON_HPP
#define MAP_TO_GRAPH_MAPPOLYGON_HPP

#include <vector>
#include <string>
#include <array>
#include "Graph.hpp"

class MapPolygon {
private:
    std::vector<std::pair<double, double>> polygon_points;
    double smallest_x, biggest_x, smallest_y, biggest_y;
    bool points_loaded = false;

    /*!
     * Add point from a string representation of it in format "x,y,z"
     * @param point_string String representation fo a point
     */
    void add_point(const std::string &point_string);

    /*!
     * Add points from a string of a format "x,y,z x2,y2,z2 ..."
     * @param kml_file_string String, representing the points of a polygon
     * @return true if at least one point from the string was added successfully
     */
    bool add_points_from_string(std::string kml_file_string);

public:
    bool load_polygon_from_file(const std::string &filename);

    /*!
     * Generate a grid of points that belong to the polygon.
     * @param x_step Horizontal step of the grid
     * @param y_step Vertical step of the grid
     * @return Vector of points that belong to the monitor
     */
    Graph points_inside_polygon(double step);


};


#endif //MAP_TO_GRAPH_MAPPOLYGON_HPP
