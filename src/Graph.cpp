//
// Created by mrs on 24.01.22.
//
#include <iostream>
#include "Graph.hpp"
#include <algorithm>
#include "utils.hpp"

Graph::Graph(const MapPolygon &map_polygon, double rotation_angle, double step): m_rotation_angle(rotation_angle), m_step(step) {

    // Make the rotated map
    auto map_polygon_rotated = map_polygon.rotated(m_rotation_angle);

    // Get the smallest x and y coordinates of all the MapPolygon points
    auto smallest = map_polygon_rotated.reduce_points([](const auto &p1, const auto &p2){return std::make_pair(std::min(p1.first, p2.first), std::min(p1.second, p2.second));});
    auto biggest = map_polygon_rotated.reduce_points([](const auto &p1, const auto &p2){return std::make_pair(std::max(p1.first, p2.first), std::max(p1.second, p2.second));});
    m_smallest_x = smallest.first; m_smallest_y = smallest.second;
    double biggest_x = biggest.first, biggest_y = biggest.second;

    m_width = static_cast<size_t>((biggest_x - m_smallest_x) / step) + 2;
    m_height = static_cast<size_t>((biggest_y - m_smallest_y) / step) + 2;

    m_vertices_matrix = std::vector<bool>(m_width * m_height, false);

    double x, y;
    // Iterate through each row of the grid
    for (int row = 0; (y = row * step + m_smallest_y + (step / 2)) < biggest_y; row++) {
        // Now, for simplicity assume that boundaries of each zone (both fly and no-fly) never intersect
        std::vector<double> segments_intersection_x_coord;

        // Find out all the x coordinates of points of the intersection of horizontal line with any zones boundaries
        auto polygon_segments = map_polygon_rotated.get_all_segments();

        for (const auto &segment: polygon_segments) {
            if ((y <= segment.first.second && y >= segment.second.second) ||
                    (y >= segment.first.second && y <= segment.second.second)) {

                double intersection_x = segment.second.first - ((segment.second.second - y) * (segment.second.first - segment.first.first)) /
                        (segment.second.second - segment.first.second);
                segments_intersection_x_coord.push_back(intersection_x);
            }
        }
        // Sort all the x coordinates
        // Now all the points inside the polygon are between intersection 0 and 1, 2 and 3, 3 and 4 and so on
        std::sort(segments_intersection_x_coord.begin(), segments_intersection_x_coord.end());
        if (segments_intersection_x_coord.size() % 2 == 0 and !segments_intersection_x_coord.empty()) {
            size_t intersections_passed = 0;
            for (int col = 0; (x = col * step + m_smallest_x + step / 2) < biggest_x; col++) {
                while (intersections_passed < segments_intersection_x_coord.size() and
                       segments_intersection_x_coord[intersections_passed] < x) {
                    intersections_passed++;
                }
                if (intersections_passed % 2 == 1) {
                    add_vertex(row, col);
                }
            }
        }
    }

}

bool Graph::operator()(size_t row, size_t col) const {
    return m_vertices_matrix[row * m_width + col];
}

void Graph::add_vertex(int row, int col) {
    m_vertices_matrix[row * m_width + col] = true;
}

size_t Graph::get_height() const {
    return m_height;
}

size_t Graph::get_width() const {
    return m_width;
}

double Graph::get_step() const {
    return m_step;
}

std::pair<double, double> Graph::get_point_coords(size_t row, size_t col) const {
    // Calculate the coordinates of point and rotate it by the initial angle in the opposite direction
    return rotate_point({static_cast<double>(col) * m_step + m_smallest_x + (m_step / 2),
                         static_cast<double>(row) * m_step + m_smallest_y + (m_step / 2)}, -m_rotation_angle);
}


