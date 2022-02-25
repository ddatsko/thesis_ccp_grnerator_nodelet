#include "MapPolygon.hpp"
#include <pugixml.hpp>
#include <iostream>
#include "utils.hpp"
#include <algorithm>
#include <stdexcept>
#include "Graph.hpp"
#include <ros/ros.h>

void MapPolygon::load_polygon_from_file(const std::string &filename) {
    points_loaded = false;
    bool edge_values_updated = false;
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

    for (auto placemark: document.children("Placemark")) {
        auto placemark_polygon_points = get_points_from_string(
                placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").text().as_string());

        for (auto &point: placemark_polygon_points) {
            // Update smallest and biggest values
            if (not edge_values_updated) {
                edge_values_updated = true;
                smallest_x = biggest_x = point.first;
                smallest_y = biggest_y = point.second;
            } else {
                smallest_x = std::min(smallest_x, point.first);
                biggest_x = std::max(biggest_x, point.first);
                smallest_y = std::min(smallest_y, point.second);
                biggest_y = std::max(biggest_y, point.second);
            }
        }
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
            std::cout << "Non fly zone" << std::endl;
            no_fly_zone_polygons.push_back(placemark_polygon_points);
        }
        
    }
    if (not fly_zone_found) {
      throw kml_file_parse_error("Could not find fly zone in the kml file");
    }
    points_loaded = true;
}


MapPolygon::polygon_t MapPolygon::get_points_from_string(std::string kml_file_string) {
    polygon_t points;

    // Split the string by spaces and add each point to the vector of internal points
    size_t pos;
    while ((pos = kml_file_string.find(' ')) != std::string::npos) {
        std::string point_string = kml_file_string.substr(0, pos);
        kml_file_string.erase(0, pos + 1);
        if (point_string.empty()) {
            continue;
        }
        points.push_back(string_to_point(point_string));
    }
    return points;
}

std::pair<double, double> MapPolygon::string_to_point(const std::string &point_string) {
    size_t coma_pos;
    std::pair<double, double> point;
    try {
        point.first = std::stod(point_string, &coma_pos);
        point.second = std::stod(point_string.substr(coma_pos + 1));
    } catch (std::invalid_argument &e) {
        throw kml_file_parse_error{"Could not convert string too coordinates..."};
    }
    return point;
}


Graph MapPolygon::points_inside_polygon(double step) {

    Graph g(smallest_x, biggest_x, smallest_y, biggest_y, step);
    // Return the empty vector if the points are not still loaded or arguments are invalid
    if ((not points_loaded) or step < 0) {
        return g;
    }
    // Iterate through each row of the grid
    for (int row = 0; row * step + smallest_y < biggest_y; row++) {
        double y = smallest_y + row * step;
        // Now, for simplicity assume that boundaries of each zone (both fly and no-fly) never intersect
        std::vector<double> segments_intersection_x_coord;
        hom_t horizontal_line = {0, 1, -y};

        // TODO: rewrite this without copying of the code
        // Find out all the x coordinates of points of the intersection of horizontal line with any zones boundaries
        for (size_t i = 0; i < fly_zone_polygon_points.size() - 1; i++) {
            auto intersection_x = segment_line_intersection(fly_zone_polygon_points[i], fly_zone_polygon_points[i + 1], horizontal_line).first;
            if ((intersection_x <= fly_zone_polygon_points[i].first and
                 intersection_x >= fly_zone_polygon_points[i + 1].first) or
                (intersection_x <= fly_zone_polygon_points[i + 1].first and
                 intersection_x >= fly_zone_polygon_points[i].first)) {

                segments_intersection_x_coord.push_back(intersection_x);
            }
        }

        // Find out all the intersections with no-fly zone polygons
        for (auto &no_fly_zone_polygon: no_fly_zone_polygons) {
            for (size_t i = 0; i < no_fly_zone_polygon.size() - 1; i++) {
                auto intersection_x = segment_line_intersection(no_fly_zone_polygon[i], no_fly_zone_polygon[i + 1], horizontal_line).first;
                if ((intersection_x <= no_fly_zone_polygon[i].first and
                     intersection_x >= no_fly_zone_polygon[i + 1].first) or
                    (intersection_x <= no_fly_zone_polygon[i + 1].first and
                     intersection_x >= no_fly_zone_polygon[i].first)) {

                    segments_intersection_x_coord.push_back(intersection_x);
                }
            }
        }
        // Sort all the x coordinates
        // Now all the points inside the polygon are between intersection 0 and 1, 2 and 3, 3 and 4 and so on
        std::sort(segments_intersection_x_coord.begin(), segments_intersection_x_coord.end());
        if (segments_intersection_x_coord.size() % 2 == 0 and !segments_intersection_x_coord.empty()) {
            size_t intersections_passed = 0;
            double x;
            for (int col = 0; (x = col * step + smallest_x) < biggest_x; col++) {
                while (intersections_passed < segments_intersection_x_coord.size() and
                       segments_intersection_x_coord[intersections_passed] < x) {
                    intersections_passed++;
                }
                if (intersections_passed % 2 == 1) {
                    g.add_vertex(row, col);
                }
            }
        }

    }
    return g;
}

