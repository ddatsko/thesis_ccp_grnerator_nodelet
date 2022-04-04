//
// Created by mrs on 23.03.22.
//

#include "ShortestPathCalculator.hpp"
#include <algorithm>
#include "utils.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>

namespace {
    const double EPS = 1e-7;

    point_t pm(point_t point) {
        return {point.first * 1000, point.second * 1000};
    }

    std::set<segment_t> no_direct_path(const std::vector<point_t> &polygon, bool fly_zone = true) {
        // Vector of turning angles around each polygon node
        std::vector<double> angles;
//        std::cout << "Size: " << polygon.size() << std::endl;
        angles.push_back(angle_between_vectors(pm(polygon[polygon.size() - 2]), pm(polygon[0]), pm(polygon[1])));
        for (size_t i = 1; i + 1 < polygon.size(); ++i) {
//            std::cout << i << std::endl;
            angles.push_back(angle_between_vectors(pm(polygon[i - 1]), pm(polygon[i]), pm(polygon[i + 1])));
        }

//        std::cout << "Angles: " << std::endl;
//        for (auto &a: angles) {
//            std::cout << a / M_PI * 180 << std::endl;
//        }
//        std::cout << "---------------------------------" << std::endl;

        std::set<segment_t> no_direct_view;
        // Iterate through each pair of points in the polygon and if the sum of angles between them is not suitable -
        // add them to the list of not possible ones
        for (size_t i = 0; i < angles.size(); ++i) {
            double angle = 0;
            for (size_t j = i + 2; j < angles.size(); j++) {
                angle += M_PI - angles[j - 1];
                if (((angle < 0 - EPS || angle > 2 * M_PI + EPS) && fly_zone) ||
                    ((angle < 2 * M_PI - EPS || angle > 0 + EPS) && !fly_zone)) {
                    no_direct_view.insert({polygon[i], polygon[j]});
                }
            }
        }
        // If first and last points were added accidentally, remove them
        auto pos = no_direct_view.find({polygon.front(), polygon[polygon.size() - 2]});
        if (pos != no_direct_view.end()) {
            no_direct_view.erase(pos);
        }
        return no_direct_view;
    }

    /*!
     * Calculate the segments, between which there is no direct path definitely.
     * Needed for the main algorithm while checking if there is a direct path by
     * "visibility" of the points (no segment between them). But this situation can also
     * happen when nodes "see" each other through a no-fly zone (e.g. different points of a convex no-fly zone
     * of when the fly-zone if non-convex, so there always exist a line between
     * @param polygon Polygon for which segments will be found
     * @return
     */
    std::set<segment_t> no_direct_path(const MapPolygon &polygon) {
        std::set<segment_t> no_direct_view = no_direct_path(polygon.fly_zone_polygon_points, true);
        for (const auto &p: polygon.no_fly_zone_polygons) {
            auto no_fly_zone_no_view = no_direct_path(p, false);
            no_direct_view.insert(no_fly_zone_no_view.begin(), no_fly_zone_no_view.end());
        }
        return no_direct_view;
    }
}

ShortestPathCalculator::ShortestPathCalculator(const MapPolygon &polygon) {
    auto points_tmp = polygon.get_all_points();
    m_polygon_points = std::vector<point_t>{points_tmp.begin(), points_tmp.end()};
    m_polygon_segments = polygon.get_all_segments();

    // Assign each point a unique identifier to be able to quickly traverse through it
    int index = 0;
    for (const auto &p: m_polygon_points) {
        m_point_index[p] = index++;
    }

    // Create a 2-d matrix that will contain paths for the shortest path between each of perimeter point
    // And 2-d matrix for the Floyd-Warshall algorithm
    m_next_vertex_in_path = std::vector<std::vector<size_t>>(m_polygon_points.size());
    std::for_each(m_next_vertex_in_path.begin(), m_next_vertex_in_path.end(),
                  [&](auto &row) { row = std::vector<size_t>(m_polygon_points.size(), -1); });
    m_floyd_warshall_d = std::vector<std::vector<double>>(m_polygon_points.size());
    std::for_each(m_floyd_warshall_d.begin(), m_floyd_warshall_d.end(),
                  [&](auto &row){row = std::vector<double>(m_polygon_points.size(), HUGE_VAL);});

    // Get the list of pairs of points, for which there is no direct path even if they can "see" each other
    auto no_direct_path_pairs = no_direct_path(polygon);

//    std::cout << "No pairs: " << std::endl;
//    for (auto &p: no_direct_path_pairs) {
//        std::cout << p.first.first << "," << p.first.second << "   " << p.second.first << "," << p.second.second << std::endl;
//    }
//    std::cout << "------------------------" << std::endl;

    // O(N^3) building of the matrix. Ok as the algorithm itself runs on O(N^3)
    for (size_t i = 0; i < m_polygon_points.size(); i++) {
        m_floyd_warshall_d[i][i] = 0;
        for (size_t j = i + 1; j < m_polygon_points.size(); j++) {
            segment_t segment_between_point = {m_polygon_points[i], m_polygon_points[j]};
            if (m_polygon_points[i] == m_polygon_points[j]) {
                continue;
            }
//            std::cout << "Checking points: " << segment_between_point.first.first << "," << segment_between_point.first.second << "   " <<
//            segment_between_point.second.first << "," << segment_between_point.second.second << std::endl;

            // If there is no direct path by previously calculated algorithm, omit this pair
            if (no_direct_path_pairs.find(segment_between_point) != no_direct_path_pairs.end() ||
                no_direct_path_pairs.find({segment_between_point.second, segment_between_point.first}) != no_direct_path_pairs.end()) {
//                std::cout << "No direct path" << std::endl;
                continue;
            }

            bool segment_intersects = false;
            for (const auto &segment: m_polygon_segments) {
//                std::cout << "Segment: " << segment.first.first << "," << segment.first.second << "   " <<
//                segment.second.first << "," << segment.second.second << std::endl;

                // If there is an exact edge between these two points -- break as there won't be any intersections anymore
                if ((segment.first == segment_between_point.first && segment.second == segment_between_point.second) ||
                    (segment.second == segment_between_point.first && segment.first == segment_between_point.second)) {
                    segment_intersects = false;
//                    std::cout << "Exact edge between point " << std::endl;
                    break;
                }

                // If the segment starts in point - don't check it for intersection as it will intersect in that point
                if (segment.first == segment_between_point.first || segment.second == segment_between_point.first ||
                    segment.first == segment_between_point.second || segment.second == segment_between_point.second) {
                    continue;
                }

                if (segments_intersect(segment, segment_between_point)) {
                    segment_intersects = true;
                    break;
                }
            }
            if (!segment_intersects) {
                // Update Floyd Warshall algorithm matrix
                m_floyd_warshall_d[i][j] = m_floyd_warshall_d[j][i] = segment_length(segment_between_point);
                m_next_vertex_in_path[i][j] = j;
                m_next_vertex_in_path[j][i] = i;
            }
        }
    }
    /*
    for (size_t i = 0; i < polygon_points.size(); ++i) {
        std::cout << i << ": " << polygon_points[i].first << ", " << polygon_points[i].second << std::endl;
    }

    for (size_t i = 0; i < polygon_points.size(); ++i) {
        for (size_t j = 0; j < polygon_points.size(); j++) {
            std::cout << (m_floyd_warshall_d[i][j] == HUGE_VAL ? 0 : 1) << " ";
        }
        std::cout << std::endl;
    }
     */
    run_floyd_warshall();
}


void ShortestPathCalculator::run_floyd_warshall() {
    const size_t N = m_floyd_warshall_d.size();
    for (size_t k = 0; k < N; ++k) {
        for (size_t j = 0; j < N; ++j) {
            for (size_t i = 0; i < N; ++i) {
                if (m_floyd_warshall_d[i][j] > m_floyd_warshall_d[i][k] + m_floyd_warshall_d[k][j]) {
                    m_floyd_warshall_d[i][j] = m_floyd_warshall_d[i][k] + m_floyd_warshall_d[k][j];
                    m_floyd_warshall_d[j][i] = m_floyd_warshall_d[i][j];
                    // Now, to get to j from i, we should in direction to k (to next vertex in path to k)
                    m_next_vertex_in_path[i][j] = m_next_vertex_in_path[i][k];
                    m_next_vertex_in_path[j][i] = m_next_vertex_in_path[j][k];
                }
            }
        }
    }
}

std::vector<point_t> ShortestPathCalculator::get_approximate_shortest_path(point_t p1, point_t p2) const {
    if (point_can_see_point(p1, p2)) {
        return std::vector<point_t> {p1, p2};
    }

    point_t closest_to_start = closest_polygon_point(p1);
    point_t closest_to_end = closest_polygon_point(p2);

    auto path_between_closest = shortest_path_between_polygon_nodes(m_point_index[closest_to_start], m_point_index[closest_to_end]);
    // TODO: can check if the second path point can be reached directly from p1 to make path feasible

    path_between_closest.insert(path_between_closest.begin(), p1);
    path_between_closest.push_back(p2);
    return path_between_closest;

}

point_t ShortestPathCalculator::closest_polygon_point(point_t p) const {
    if (m_polygon_points.empty()) {
        throw shortest_path_calculation_error("No point in the polygon found");
    }
    point_t closest_point = m_point_index.begin()->first;
    double closest_distance = distance_between_points(closest_point, p);
    for (const auto &point: m_polygon_points) {
        double new_distance = distance_between_points(point, p);
        if (new_distance < closest_distance) {
            closest_distance = new_distance;
            closest_point = point;
        }
    }
    return closest_point;
}


std::vector<point_t> ShortestPathCalculator::shortest_path_between_polygon_nodes(size_t i, size_t j) const {
    std::vector<point_t> res;
    while (i != j) {
        res.push_back(m_polygon_points[i]);
        i = m_next_vertex_in_path[i][j];
    }
    res.push_back(m_polygon_points[j]);
    return res;
}

std::vector<point_t> ShortestPathCalculator::shortest_path_between_points(point_t p1, point_t p2) const {
    // TODO: implement the real algorithm for shortest path (will be O(n^2), i guess)
    // Now, again not exact one, but pretty good (O(N^2), but actually O(approximate_path_length * N)
    auto path = get_approximate_shortest_path(p1, p2);

    // If we can skip visiting the first point in path -- do it
    if (path[1] != p2 && point_can_see_point(p1, path[2])) {
        path.erase(path.begin() + 1);
    }
    if (path.size() > 2 && path[path.size() - 2] != p1 && point_can_see_point(p2, path[path.size() - 3])) {
        path.erase(path.begin() + static_cast<long>(path.size() - 2));
    }

    return path;
//    // Find the last point in path, seen from the first point and erase all the path up to that point
//    size_t start_ind = farthest_point_seen_in_path(p1, path);
//    path.erase(path.begin(), path.begin() + static_cast<long>(start_ind));
//
//    // Reverse and do the same trick with the last point
//    std::reverse(path.begin(), path.end());
//    start_ind = farthest_point_seen_in_path(p2, path);
//    path.erase(path.begin(), path.begin() + static_cast<long>(start_ind));
//
//    // Add the first and last point to the path as they were definitely erased
//    path.push_back(p1);
//    std::reverse(path.begin(), path.end());
//    path.push_back(p2);
//    return path;
}

size_t ShortestPathCalculator::farthest_point_seen_in_path(point_t source_point, const std::vector<point_t> &path) const {
    size_t res = 0;
    for (size_t i = 0; i < path.size(); ++i) {
        if (point_can_see_point(source_point, path[i])) {
            res = i;
        }
    }
    return res;
}

bool ShortestPathCalculator::point_can_see_point(point_t p1, point_t p2) const{
    segment_t segment{p1, p2};
    for (const auto &border_segment: m_polygon_segments) {
        if (segments_intersect(segment, border_segment)) {
            return false;
        }
    }
    return true;
}







