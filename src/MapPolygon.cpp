#include "MapPolygon.hpp"
#include <pugixml.hpp>
#include <iostream>
#include "utils.hpp"
#include <algorithm>
#include <stdexcept>
#include <ros/ros.h>



namespace {
    std::pair<double, double> string_to_point(const std::string &point_string) {
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

    MapPolygon::polygon_t get_points_from_string(std::string kml_file_string) {
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
                placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").text().as_string());

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
                  [angle](const auto &polygon){
        polygon_t no_fly_new_polygon;
        std::transform(polygon.begin(),
                      polygon.end(),
                      std::inserter(no_fly_new_polygon, no_fly_new_polygon.begin()),
                      [angle](const auto &p){return rotate_point(p, angle);});
        return no_fly_new_polygon;
    });
    return new_polygon;
}
