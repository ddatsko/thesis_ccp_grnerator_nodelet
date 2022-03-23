#ifndef MAP_TO_GRAPH_MAPPOLYGON_HPP
#define MAP_TO_GRAPH_MAPPOLYGON_HPP

#include <vector>
#include <string>
#include <array>
#include <stdexcept>
#include <numeric>
#include <set>
#include <map>
#include "custom_types.hpp"


struct kml_file_parse_error: public std::runtime_error {
  using runtime_error::runtime_error;
};

struct non_existing_point_error: public std::runtime_error {
    using runtime_error::runtime_error;
};

struct wrong_polygon_format_error: public std::runtime_error {
    using runtime_error::runtime_error;
};


struct MapPolygon {
    using polygon_t = std::vector<point_t>;

    const std::string FLY_ZONE_PLACEMARK_NAME = "fly-zone";
    const std::string NO_FLY_ZONE_PLACEMARK_NAME = "no-fly-zone";

    polygon_t fly_zone_polygon_points;
    std::vector<polygon_t> no_fly_zone_polygons;

    MapPolygon() = default;
    MapPolygon(const MapPolygon &p);

    void load_polygon_from_file(const std::string &filename);

    // TODO: check if get_all_points method should be a method or a separate function
    /*!
     * Get the list of all the polygons point (both fly-zone and non-fly zone)
     * @return Set of all the points
     */
    [[nodiscard]] std::vector<point_t> get_all_points() const;

    /*!
     * Get the vector of all segments (of both fly and no-fly zones of the map polygon)
     * @return Vector of all the present segments
     */
    [[nodiscart]] std::vector<segment_t> get_all_segments() const;

    /*!
     * Get two neighbouring points in some polygon for the specified point
     * @param point point that belongs to one of polygons
     * @return pair of two points - neighbors of the points
     */
    [[nodiscard]] std::pair<point_t, point_t> point_neighbors(point_t point) const;

    /*!
     * Get the map polygon, rotated by angle radians counter clockwise around the coordinates origin
     * @param angle Angle of the rotation [rad]
     * @return New MapPolygon with all the points rotated by the angle
     */
    [[nodiscard]] MapPolygon rotated(double angle) const;

    /*!
     * Find the edge starting from the first rightmost point
     * @return Edge starting from the first rightmost point
     */
    [[nodiscard]] std::pair<point_t, point_t> rightmost_edge();


    //TODO: implement reducing all the points and not only ones of the outer perimeter
template<class Op>
        point_t reduce_points(Op op) {
            return std::accumulate(fly_zone_polygon_points.begin(),
                               fly_zone_polygon_points.end(),
                               *(fly_zone_polygon_points.begin()),
                               op);
        }
};


#endif //MAP_TO_GRAPH_MAPPOLYGON_HPP
