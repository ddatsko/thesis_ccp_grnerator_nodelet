#include "MapPolygon.hpp"
#include <pugixml.hpp>
#include <iostream>
#include "utils.hpp"
#include <algorithm>
#include <stdexcept>
#include <ros/ros.h>
#include "algorithms.hpp"

//TODO: make most of methods external functions (maybe, working on not MapPolygons byt ust on vector<pair<double, double>>
namespace {
    const std::string FLY_ZONE_PLACEMARK_NAME = "fly-zone";
    const std::string NO_FLY_ZONE_PLACEMARK_NAME = "no-fly-zone";

    const double SPLIT_BIN_SEARCH_THRESH = 1e-5;

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

std::pair<MapPolygon, MapPolygon> MapPolygon::split_by_vertical_line(double x) {
    make_pure_convex();
    make_polygon_clockwise(fly_zone_polygon_points);
    auto fly_zone_copy = fly_zone_polygon_points;
    fly_zone_copy.pop_back();
    std::vector<std::pair<double, double>> new_pol[2];
    double rightmost_x[2] = {std::numeric_limits<double>::min(), std::numeric_limits<double>::min()};
    int cur_pol = 0;

    // Iterate through each segment, detecting crossing of the vertical line
    for (size_t i = 0; i < fly_zone_copy.size(); ++i) {
        size_t next = (i + 1) % fly_zone_copy.size();
        new_pol[cur_pol].push_back(fly_zone_copy[i]);
        if (x >= std::min(fly_zone_copy[i].first, fly_zone_copy[next].first) && x <= std::max(fly_zone_copy[i].first, fly_zone_copy[next].first)) {
            // If vertical line is crossed - change the polygon for pushing points to and push intersection points
            auto intersection = segment_line_intersection(fly_zone_copy[i], fly_zone_copy[next], {1, 0, -x});
            new_pol[cur_pol].push_back(intersection);
            cur_pol ^= 1;
            new_pol[cur_pol].push_back(intersection);
        }
        for (int j = 0; j < 2; ++j) {
            if (!new_pol[j].empty()) {
                rightmost_x[j] = std::max(rightmost_x[j], new_pol[j].back().first);
            }
        }
    }
    new_pol[cur_pol].push_back(fly_zone_copy.back());
    // Add starting point to make the polygon closed
    new_pol[0].push_back(new_pol[0].front());
    new_pol[1].push_back(new_pol[1].front());
    std::pair<MapPolygon, MapPolygon> res;

    // Place the polygon on the left to the left
    if (rightmost_x[0] < rightmost_x[1]) {
        res.first.fly_zone_polygon_points = new_pol[0];
        res.second.fly_zone_polygon_points = new_pol[1];
    } else {
        res.first.fly_zone_polygon_points = new_pol[1];
        res.second.fly_zone_polygon_points = new_pol[0];
    }
    return res;
}




std::vector<MapPolygon> MapPolygon::split_into_pieces(double max_piece_area) {
    if (max_piece_area > area()) {
        return {*this};
    }
    make_polygon_clockwise(fly_zone_polygon_points);

    int res_polygons = std::ceil(area() / max_piece_area);
    double split_piece_area = area() / res_polygons;
    auto cur_polygon = *this;
    std::vector<MapPolygon> res;
    double rightmost_x = std::numeric_limits<double>::min(), leftmost_x = std::numeric_limits<double>::max();
    for (const auto &p: fly_zone_polygon_points) {
        rightmost_x = std::max(rightmost_x, p.first);
        leftmost_x = std::min(leftmost_x, p.first);
    }

    for (int i = 0; i < res_polygons - 1; ++i) {
        // Using bin-search find the appropriate vertical line
        double l = leftmost_x, r = rightmost_x;
        while (r - l > SPLIT_BIN_SEARCH_THRESH) {
            double m = (r + l) / 2;
            auto line_split = cur_polygon.split_by_vertical_line(m);
            if (line_split.first.area() < split_piece_area) {
                l = m;
            } else {
                r = m;
            }
        }
        // Take a piece from the left and continue the algorithm
        auto best_split = cur_polygon.split_by_vertical_line(l);
        res.push_back(best_split.first);
        cur_polygon = best_split.second;
        leftmost_x = l;
    }
    res.push_back(cur_polygon);
    return res;
}
