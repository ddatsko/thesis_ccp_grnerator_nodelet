#include "MapPolygon.hpp"
#include <pugixml.hpp>
#include <iostream>
#include "utils.hpp"
#include <algorithm>
#include <stdexcept>
#include <ros/ros.h>
#include "algorithms.hpp"


namespace {
    const std::string FLY_ZONE_PLACEMARK_NAME = "fly-zone";
    const std::string NO_FLY_ZONE_PLACEMARK_NAME = "no-fly-zone";

    point_t string_to_point(const std::string &point_string) {
        size_t coma_pos;
        point_t point;
        try {
            point.first = std::stod(point_string, &coma_pos);
            point.second = std::stod(point_string.substr(coma_pos + 1));
        } catch (std::invalid_argument &e) {
            throw kml_file_parse_error{"Could not convert string '" + point_string + "' too coordinates..."};
        }
        return point;
    }

    /*!
     * Make the polygon be directed clockwise (when travelling from the first to last point,
     * the area inside of polygon is always on the right) for the convenience of working with it
     */
    void make_polygon_clockwise(polygon_t &polygon) {
        if (polygon.size() <= 2) {
            return;
        }
        double angle = M_PI - angle_between_vectors(polygon[polygon.size() - 2], polygon[0], polygon[1]);
        for (size_t i = 0; i + 2 < polygon.size(); ++i) {
            angle += M_PI - angle_between_vectors(polygon[i], polygon[i + 1], polygon[i + 2]);
        }
        // If the outer angle of the whole polygon is -2 * pi -- we need to reverse it
        if (std::abs(-2 * M_PI - angle) < std::abs(2 * M_PI - angle)) {
            std::reverse(polygon.begin(), polygon.end());
        }
    }
}

MapPolygon::polygon_t get_points_from_string(std::string kml_file_string) {
    // Strip the string
    kml_file_string.erase(kml_file_string.find_last_not_of(" \t\n\r\f\v") + 1);
    kml_file_string.erase(0, kml_file_string.find_first_not_of(" \t\n\r\f\v"));

    polygon_t points;

    // Split the string by spaces and add each point to the vector of internal points
    size_t pos;
    while (!kml_file_string.empty()) {
        pos = kml_file_string.find(' ');
        std::string point_string = kml_file_string.substr(0, pos);
        if (pos == std::string::npos) {
            kml_file_string = std::string();
        } else {
            kml_file_string.erase(0, pos + 1);
        }
        if (point_string.empty()) {
            continue;
        }
        points.push_back(gps_coordinates_to_meters(string_to_point(point_string)));
//        points.push_back(string_to_point(point_string));
    }

    make_polygon_clockwise(points);
    return points;
}

MapPolygon::MapPolygon(const MapPolygon &p) {
    fly_zone_polygon_points = p.fly_zone_polygon_points;
    no_fly_zone_polygons = p.no_fly_zone_polygons;
}

void MapPolygon::load_polygon_from_file(const std::string &filename) {
    bool fly_zone_found = false;

    // Try to load the file with described polygon
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
        throw kml_file_parse_error("Could not load kml file..");
    }

    auto document = doc.child("kml").child("Document");
    if (document.empty()) {
        throw kml_file_parse_error("Could not find any polygon in the kml file.");
    }

    for (const auto &placemark: document.children("Placemark")) {
        auto placemark_polygon_points = get_points_from_string(
                placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child(
                        "coordinates").text().as_string());
        if (placemark.child("name").text().as_string() == FLY_ZONE_PLACEMARK_NAME) {
            fly_zone_found = true;
            fly_zone_polygon_points = placemark_polygon_points;
            if (fly_zone_polygon_points.empty()) {
                throw kml_file_parse_error("Could not load fly zone polygon points");
            }
        } else if (placemark.child("name").text().as_string() == NO_FLY_ZONE_PLACEMARK_NAME) {
            if (placemark_polygon_points.empty()) {
                throw kml_file_parse_error("Could not load no fly zone polygon...");
            }
//            std::cout << "Non fly zone" << std::endl;
            no_fly_zone_polygons.push_back(placemark_polygon_points);
        }

    }
    if (not fly_zone_found) {
        throw kml_file_parse_error("Could not find fly zone in the kml file");
    }
}


MapPolygon MapPolygon::rotated(double angle) const {
    // Make a copy of this mapPolygon
    MapPolygon new_polygon;


    // Rotate each point of the polygon describing the fly-zone
    std::transform(fly_zone_polygon_points.begin(),
                   fly_zone_polygon_points.end(),
                   std::inserter(new_polygon.fly_zone_polygon_points, new_polygon.fly_zone_polygon_points.begin()),
                   ([angle](const auto &p) { return rotate_point(p, angle); }));


    // Rotate each point in each of the polygons describing non-fly zone
    std::transform(no_fly_zone_polygons.begin(),
                   no_fly_zone_polygons.end(),
                   std::inserter(new_polygon.no_fly_zone_polygons, new_polygon.no_fly_zone_polygons.begin()),
                   [angle](const auto &polygon) {
                       polygon_t no_fly_new_polygon;
                       std::transform(polygon.begin(),
                                      polygon.end(),
                                      std::inserter(no_fly_new_polygon, no_fly_new_polygon.begin()),
                                      [angle](const auto &p) { return rotate_point(p, angle); });
                       return no_fly_new_polygon;
                   });
    return new_polygon;
}


std::set<point_t> MapPolygon::get_all_points() const {
    std::set<point_t> points;
    std::copy(fly_zone_polygon_points.begin(), fly_zone_polygon_points.end(),
              std::inserter(points, points.begin()));
    std::for_each(no_fly_zone_polygons.begin(), no_fly_zone_polygons.end(),
                  [&](const auto &p) {
                      std::copy(p.begin(), p.end(), std::inserter(points, points.begin()));
                  });
    return points;
}

namespace {
    template<class T>
    bool find_point_neighbours(point_t point, std::pair<point_t, point_t> &res, const T &container) {
        if (container.size() < 3) {
            throw wrong_polygon_format_error("Polygon has less than 3 points");
        }
        if (container[0] == point) {
            res = {container[container.size() - 2], container[1]};
            return true;
        }

        // First and last points are always the same, so skip the last one for simplicity
        for (size_t i = 1; i < container.size() - 1; ++i) {
            if (container[i] == point) {
                res = {container[i - 1], container[i + 1]};
                return true;
            }
        }
        return false;
    }
}


std::pair<point_t, point_t> MapPolygon::point_neighbors(point_t point) const {
    std::pair<point_t, point_t> res;
    if (find_point_neighbours(point, res, fly_zone_polygon_points)) {
        return res;
    }
    for (auto &p: no_fly_zone_polygons) {
        if (find_point_neighbours(point, res, p)) {
            return res;
        }
    }
    throw non_existing_point_error("Point is not in the polygon");
}

[[nodiscard]] std::pair<point_t, point_t> MapPolygon::rightmost_edge() const {
    double rightmost_x = fly_zone_polygon_points[0].first;
    for (const auto &point: fly_zone_polygon_points) {
        rightmost_x = std::max(rightmost_x, point.first);
    }
    for (size_t i = 0; i < fly_zone_polygon_points.size() - 1; ++i) {
        if (fly_zone_polygon_points[i].first == rightmost_x) {
            return {fly_zone_polygon_points[i], fly_zone_polygon_points[i + 1]};
        }
    }
    throw wrong_polygon_format_error("Cannot find a rightmost point in polygon. Probably it is empty");
}

std::vector<segment_t> MapPolygon::get_all_segments() const {
    std::vector<segment_t> segments;
    for (size_t i = 0; i + 1 < fly_zone_polygon_points.size(); ++i) {
        segments.emplace_back(fly_zone_polygon_points[i], fly_zone_polygon_points[i + 1]);
    }
    for (const auto& pol: no_fly_zone_polygons) {
        for (size_t i = 0; i + 1 < pol.size(); ++i) {
            segments.emplace_back(pol[i], pol[i + 1]);
        }
    }
    return segments;
}

std::vector<double> MapPolygon::get_n_longest_edges_rotation_angles(size_t n) const {
    auto edges = get_all_segments();
    std::sort(edges.begin(), edges.end(), [](const segment_t &s1, const segment_t &s2){return segment_length(s1) >
            segment_length(s2);});
    std::vector<double> rotations;
    for (size_t i = 0; i < n && i < edges.size(); ++i) {
        double segment_rotation = M_PI_2 - get_segment_rotation(edges[i]);
        rotations.push_back(segment_rotation);
        if (segment_rotation < M_PI) {
            rotations.push_back(segment_rotation + M_PI);
        } else {
            rotations.push_back(segment_rotation - M_PI);
        }
    }
    return rotations;
}

std::vector<MapPolygon> MapPolygon::split_into_pieces(double max_piece_area) {
    if (max_piece_area > area()) {
        return {*this};
    }
    size_t number_of_pieces = std::ceil(area() / max_piece_area);

    make_pure_convex();
    make_polygon_clockwise(fly_zone_polygon_points);

    // Find the index of the longest edge in the polygon and
    size_t longest_edge_i = 0;
    double longest_edge = std::numeric_limits<double>::min();

    for (size_t i = 0; i + 1 < fly_zone_polygon_points.size(); ++i) {
        double edge_length = segment_length({fly_zone_polygon_points[i], fly_zone_polygon_points[i + 1]});
        std::cout << "Length: " << edge_length << std::endl;
        if (edge_length > longest_edge) {
            longest_edge = edge_length;
            longest_edge_i = i;
        }
    }

    // Rotate the polygon to make the longest edge vertical
    // TODO: check if the rotation angle is appropriate and the longest edge is always at the left
    double rotation_angle = get_segment_rotation({fly_zone_polygon_points[longest_edge_i], fly_zone_polygon_points[longest_edge_i + 1]});
    std::cout << "angle: " << rotation_angle << std::endl;
    rotation_angle = -(rotation_angle + 3 * M_PI_2);
    std::cout << "rotation angle" << rotation_angle << std::endl;

    std::cout << "Not rotated polygon: " << area() << std::endl;
    for (const auto &p: fly_zone_polygon_points) {
        std::cout << "(" << std::setprecision(10) << p.first << ", " << std::setprecision(10) << p.second << ")" << std::endl;
    }

    auto polygon_rotated = rotated(rotation_angle);
    auto fly_zone = polygon_rotated.fly_zone_polygon_points;


    std::cout << "Rotated polygon: " << polygon_rotated.area() << std::endl;
    for (const auto &p: polygon_rotated.fly_zone_polygon_points) {
        std::cout << "(" << std::setprecision(10) << p.first << ", " << std::setprecision(10) << p.second << ")" << std::endl;
    }

    // Assuming that polygon is rotated clockwise and the longest edge is on the left
    size_t it1 = longest_edge_i;
    size_t it2 = longest_edge_i + 1;
    if (it2 == fly_zone.size() - 1) {
        it2 = 0;
    }
    fly_zone.pop_back();

    std::vector<MapPolygon> res;
    double current_area = 0.0;
    std::vector<point_t> upper_points {fly_zone[it2]};
    std::vector<point_t> lower_points {fly_zone[it1]};

    std::cout << "It1: " << it1 << ", It2: " << it2 << std::endl;

    segment_t upper_segment = {fly_zone[it2], fly_zone[(it2 + 1) % fly_zone.size()]};
    it2 = (it2 + 1) % fly_zone.size();
    segment_t lower_segment = {fly_zone[it1], fly_zone[(it1 + fly_zone.size() - 1) % fly_zone.size()]};
    it1 = (it1 + fly_zone.size() - 1) % fly_zone.size();

    while(it1 != it2) {
        std::cout << "Current area: " << current_area << std::endl;
        std::cout << "It1: " << it1 << ", It2: " << it2 << std::endl;
        double lower_point_x = lower_segment.second.first, upper_point_x = upper_segment.second.first;
        std::cout << "Upper point x: " << std::setprecision(10) << upper_point_x << ", Lower point x: " << std::setprecision(10) << lower_point_x << std::endl;
        if (upper_point_x < lower_point_x) {
            point_t lower_intersection = segment_line_intersection(lower_segment.first, lower_segment.second,
                                                                   {1, 0, -upper_point_x});
            upper_points.push_back(upper_segment.second);
            lower_points.push_back(lower_intersection);
            current_area += polygon_from_2_segments(upper_segment, lower_segment, upper_point_x).area();
            lower_segment.first = lower_intersection;
            it2 = (it2 + 1) % fly_zone.size();
            upper_segment = {upper_segment.second, fly_zone[it2]};
        } else {
            std::cout << "HERE" << std::endl;
            point_t upper_intersection = segment_line_intersection(upper_segment.first, upper_segment.second, {1, 0, -lower_point_x});
            std::cout << upper_intersection.first << ", " << upper_intersection.second << std::endl;

            lower_points.push_back(lower_segment.second);
            upper_points.push_back(upper_intersection);
            current_area += polygon_from_2_segments(upper_segment, lower_segment, lower_point_x).area();
            upper_segment.first = upper_intersection;
            it1 = (it1 + fly_zone.size() - 1) % fly_zone.size();
            lower_segment = {lower_segment.second, fly_zone[it1]};
        }
        if (current_area >= max_piece_area) {
            // TODO: do some nice algebra or bin-search here to make the size of new polygon be precisely max_piece_area
            MapPolygon new_polygon;
            new_polygon.fly_zone_polygon_points.insert(new_polygon.fly_zone_polygon_points.end(), upper_points.begin(), upper_points.end());
            std::reverse(lower_points.begin(), lower_points.end());
            new_polygon.fly_zone_polygon_points.insert(new_polygon.fly_zone_polygon_points.end(), lower_points.begin(), lower_points.end());
            new_polygon.fly_zone_polygon_points.push_back(upper_points.front());

            lower_points = std::vector<point_t>{lower_segment.first};
            upper_points = std::vector<point_t>{upper_segment.first};

            res.push_back(new_polygon.rotated(-rotation_angle));
        }
    }
    upper_points.push_back(fly_zone[it1]);
    MapPolygon new_polygon;
    new_polygon.fly_zone_polygon_points.insert(new_polygon.fly_zone_polygon_points.end(), upper_points.begin(), upper_points.end());
    std::reverse(lower_points.begin(), lower_points.end());
    new_polygon.fly_zone_polygon_points.insert(new_polygon.fly_zone_polygon_points.end(), lower_points.begin(), lower_points.end());
    new_polygon.fly_zone_polygon_points.push_back(upper_points.front());
    res.push_back(new_polygon.rotated(-rotation_angle));

    return res;
}


double MapPolygon::area() const {
    // TODO: implement the algorithm for non-convex polygon too
    if (!polygon_convex(this->fly_zone_polygon_points)) {
        throw std::runtime_error("THe algorithm cannot calculate the area of non convex polygon still");
    }
    double whole_area = 0;
    for (size_t i = 1; i < fly_zone_polygon_points.size(); ++i) {
        whole_area += (fly_zone_polygon_points[i - 1].first + fly_zone_polygon_points[i].first) *
                (fly_zone_polygon_points[i - 1].second - fly_zone_polygon_points[i].second);
    }
    return whole_area;
}

void MapPolygon::make_pure_convex() {
    if (fly_zone_polygon_points.empty()) {
        return;
    }
    std::vector<point_t> new_fly_zone {fly_zone_polygon_points[0]};
    for (size_t i = 1; i + 1 < fly_zone_polygon_points.size(); ++i) {
        if (std::abs(angle_between_vectors(fly_zone_polygon_points[i - 1], fly_zone_polygon_points[i], fly_zone_polygon_points[i + 1]) - M_PI) < 1e-4) {
            continue;
        }
        new_fly_zone.push_back(fly_zone_polygon_points[i]);
    }
    new_fly_zone.push_back(fly_zone_polygon_points.back());
    fly_zone_polygon_points = new_fly_zone;
}

//std::vector<MapPolygon> split_polygon(const MapPolygon &polygon,  int parts) {
//    const void* p = reinterpret_cast<const void *>(&polygon);
//
//
//}