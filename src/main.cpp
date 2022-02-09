#include <iostream>
#include "MapPolygon.hpp"
#include <iomanip>
#include "algorithms.hpp"

int maun() {
    MapPolygon polygon;
    if (not polygon.load_polygon_from_file("lake.kml")) {
        std::cerr << "Error while loading the polygon form file ..." << std::endl;
    }
    auto g = polygon.points_inside_polygon(0.00004);
    auto path = wavefront(49.5415, 15.348, g);

    for (auto &p: path) {
        std::cout << std::setprecision(10) << p.first << "," << p.second << std::endl;
    }

}