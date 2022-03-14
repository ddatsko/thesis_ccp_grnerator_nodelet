#ifndef MAP_TO_GRAPH_MAPPOLYGON_HPP
#define MAP_TO_GRAPH_MAPPOLYGON_HPP

#include <vector>
#include <string>
#include <array>
#include <stdexcept>
#include <numeric>

using polygon_t = std::vector<std::pair<double, double>>;

struct kml_file_parse_error: public std::runtime_error {
  using runtime_error::runtime_error;
};


class MapPolygon {
public:
    using polygon_t = std::vector<std::pair<double, double>>;

    const std::string FLY_ZONE_PLACEMARK_NAME = "fly-zone";
    const std::string NO_FLY_ZONE_PLACEMARK_NAME = "no-fly-zone";

    polygon_t fly_zone_polygon_points;
    std::vector<polygon_t> no_fly_zone_polygons;

    MapPolygon() = default;
    MapPolygon(const MapPolygon &p);

    void load_polygon_from_file(const std::string &filename);

    /*!
     * Get the map polygon, rotated by angle radians counter clockwise around the coordinates origin
     * @param angle Angle of the rotation [rad]
     * @return New MapPolygon with all the points rotated by the angle
     */
    MapPolygon rotated(double angle) const;


    //TODO: implement reducing all the points and not only ones of the outer perimeter
template<class Op>
        std::pair<double, double> reduce_points(Op op) {
            return std::accumulate(fly_zone_polygon_points.begin(),
                               fly_zone_polygon_points.end(),
                               *(fly_zone_polygon_points.begin()),
                               op);
        }
};


#endif //MAP_TO_GRAPH_MAPPOLYGON_HPP
