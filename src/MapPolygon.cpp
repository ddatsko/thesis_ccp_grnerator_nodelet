#include "MapPolygon.hpp"
#include <pugixml.hpp>
#include <iostream>
#include "utils.hpp"
#include <algorithm>
#include "Graph.hpp"

bool MapPolygon::load_polygon_from_file(const std::string &filename) {
    // Try to load the file with described polygon
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result) {
      return false;

    }

    // Try to find the field with polygon points coordinates
    auto polygon_coordinates = doc.child("kml").child("Document").child("Placemark").
            child("Polygon").child("outerBoundaryIs").child("LinearRing");
    if (polygon_coordinates.empty()) {
        return false;
    }

    // Iterate through polygons (in this implementation there should be just one polygon)
    for (auto tool: polygon_coordinates) {
        std::string coordinates = tool.text().as_string();
        if (!add_points_from_string(coordinates)) {
            return false;
        }
    }
    return true;
}


bool MapPolygon::add_points_from_string(std::string kml_file_string) {
    // Clear old point and set a flag that the data is not valid anymore
    points_loaded = false;
    polygon_points.clear();

    // Split the string by spaces and add each point to the vector of internal points
    size_t pos;
    while ((pos = kml_file_string.find(' ')) != std::string::npos) {
        std::string point_string = kml_file_string.substr(0, pos);
        kml_file_string.erase(0, pos + 1);
        if (point_string.empty()) {
            continue;
        }
        add_point(point_string);
    }
    return (points_loaded and polygon_points.size() > 2 and
            polygon_points[0] == polygon_points[polygon_points.size() - 1]);
}

void MapPolygon::add_point(const std::string &point_string) {
    size_t coma_pos;
    std::pair<double, double> point;
    try {
        point.first = std::stod(point_string, &coma_pos);
        point.second = std::stod(point_string.substr(coma_pos + 1));
    } catch (std::invalid_argument &e) {
        return;
    }

    // If points were not loaded still, mark them as loaded and set initial values for smallest and biggest values
    if (not points_loaded) {
        points_loaded = true;
        smallest_x = biggest_x = point.first;
        smallest_y = biggest_y = point.second;
    }

    // Update smallest and biggest coordinates
    polygon_points.push_back(point);
    smallest_x = std::min(smallest_x, point.first);
    smallest_y = std::min(smallest_y, point.second);
    biggest_x = std::max(biggest_x, point.first);
    biggest_y = std::max(biggest_y, point.second);
}


Graph MapPolygon::points_inside_polygon(double step) {
    Graph g(smallest_x, biggest_x, smallest_y, biggest_y, step);
    // Return the empty vector if the points are not still loaded or arguments are invalid
    if ((not points_loaded) or step < 0) {
        return g;
    }

    // Find all the segments of the polygon
    std::vector<hom_t> segments;
    for (size_t i = 0; i < polygon_points.size() - 1; i++) {
        hom_t p1 = {polygon_points[i].first, polygon_points[i].second, 1};
        hom_t p2 = {polygon_points[i + 1].first, polygon_points[i + 1].second, 1};
        segments.push_back(cross_product(p1, p2));
    }

    // Iterate through each row of the grid
    for (int row = 0; row * step + smallest_y < biggest_y; row++) {
        double y = smallest_y + row * step;
        std::vector<double> segments_intersection_x_coord;
        hom_t horizontal_line = {0, 1, -y};

        // Find out all the points of intersection with the boundary segments. Can be done in a more efficient
        // way if we remember the previous loop cycle information, but OK for now
        for (size_t i = 0; i < segments.size(); i++) {
            hom_t intersection_h = cross_product(segments[i], horizontal_line);

            if (std::get<2>(intersection_h) != 0) {
                double intersection_x = std::get<0>(intersection_h) / std::get<2>(intersection_h);

                if ((intersection_x <= polygon_points[i].first and intersection_x >= polygon_points[i + 1].first) or
                        (intersection_x >= polygon_points[i].first and intersection_x <= polygon_points[i + 1].first)) {

                    segments_intersection_x_coord.push_back(intersection_x);
                }
            }
        }
        // Sort all the x coordinates
        // Now all the points inside the polygon are between intersection 0 and 1, 2 amd 3, 3 amd 4 and so on
        std::sort(segments_intersection_x_coord.begin(), segments_intersection_x_coord.end());

        // Process only the even number of intersections for convenience.
        // There is much trouble if the number of intersections is odd
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

