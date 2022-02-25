#ifndef MAP_TO_GRAPH_MAPPOLYGON_HPP
#define MAP_TO_GRAPH_MAPPOLYGON_HPP

#include <vector>
#include <string>
#include <array>
#include "Graph.hpp"
#include <stdexcept>


struct kml_file_parse_error: public std::runtime_error {
  using runtime_error::runtime_error;
};


class MapPolygon {
public:
    typedef  std::vector<std::pair<double, double>> polygon_t;
    const std::string FLY_ZONE_PLACEMARK_NAME = "fly-zone";
    const std::string NO_FLY_ZONE_PLACEMARK_NAME = "no-fly-zone";

    void load_polygon_from_file(const std::string &filename);

    /*!
     * Generate a grid of points that belong to the polygon.
     * @param x_step Horizontal step of the grid
     * @param y_step Vertical step of the grid
     * @return Vector of points that belong to the monitor
     */
    Graph points_inside_polygon(double step);

    /*!
     * Get point from a string representation of it in format "x,y,z"
     * @param point_string String representation fo a point
     */
    static std::pair<double, double> string_to_point(const std::string &point_string);

    /*!
     * Get points from a string of a format "x,y,z x2,y2,z2 ..."
     * @param kml_file_string String, representing the points of a polygon
     * @return vector of points found in string
     */
    static polygon_t get_points_from_string(std::string kml_file_string);


private:
    
    polygon_t fly_zone_polygon_points;
    std::vector<polygon_t> no_fly_zone_polygons;
    
    double smallest_x, biggest_x, smallest_y, biggest_y;
    bool points_loaded = false;
};


#endif //MAP_TO_GRAPH_MAPPOLYGON_HPP
