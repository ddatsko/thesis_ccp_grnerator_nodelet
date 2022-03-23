//
// Created by mrs on 23.03.22.
//

#include "ShortestPathCalculator.hpp"
#include <algorithm>
#include "utils.hpp"
#include <cmath>

namespace {
    const double eps = 1e-7;

    /*!
     * Angle between vectors (p1, p2) and (p2, p3)
     * @param p1 First point of first vector
     * @param p2 Second point of first vector, first point of third vector
     * @param p3 Second point of the second vector
     * @return  Clockwise angle between two vectors in range (0, 2 * pi)
     */
    double angle_between_vectors(point_t p1, point_t p2, point_t p3) {
        double x1 = p2.first - p1.first, y1 = p2.second - p1.second;
        double x2 = p3.first - p2.first, y2 = p3.second - p2.second;
        double dot = x1 * x2 + y1 * y2, det = x1 * y2 - y1 * x2;
        double angle = atan2(det, dot);
        if (angle < 0) {
            angle += 2 * M_PI;
        }
        return angle;
    }

    std::set<segment_t> no_direct_path(const std::vector<point_t> &polygon, bool fly_zone = true) {
        // Vector of turning angles around each polygon node
        std::vector<double> angles;
        angles.push_back(angle_between_vectors(polygon[polygon.size() - 2], polygon[0], polygon[1]));
        for (size_t i = 1; i + 1 < polygon.size(); ++i) {
            angles.push_back(angle_between_vectors(polygon[i - 1], polygon[i], polygon[i + 1]));
        }
        std::set<segment_t> no_direct_view;
        // Iterate through each pair of points in the polygon and if the sum of angles between them is not suitable -
        // add them to the list of not possible ones
        for (size_t i = 0; i < angles.size(); ++i) {
            double angle = 0;
            for (size_t j = i + 2; j < angles.size(); j++) {
                angle += M_PI - angles[j - 1];
                if (((angle < 0 - eps || angle > 2 * M_PI + eps) && fly_zone) ||
                    ((angle < 2 * M_PI - eps || angle > 0 + eps) && !fly_zone)) {
                    no_direct_view.insert({polygon[i], polygon[j]});
                }
            }
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
    auto polygon_points = polygon.get_all_points();
    auto polygon_segments = polygon.get_all_segments();

    // Assign each point a unique identifier to be able to quickly traverse through it
    int index = 0;
    for (const auto &p: polygon_points) {
        m_point_index[p] = index++;
    }

    // Create a 2-d matrix that will contain paths for the shortest path between each of perimeter point
    // And 2-d matrix for the Floyd-Warshall algorithm
    m_distances = std::vector<std::vector<int>>(polygon_points.size());
    std::for_each(m_distances.begin(), m_distances.end(),
                  [&](auto &row) { row = std::vector<int>(polygon_points.size()); });
    auto fw_d = m_distances;

    // O(N^3) building of the matrix. Ok as the algorithm itself runs on O(N^3)
    for (size_t i = 0; i < polygon_points.size(); i++) {
        for (size_t j = i + 1; j < polygon_points.size(); j++) {
            segment_t segment_between_point = {polygon_points[i], polygon_points[j]};
            bool segment_intersects = false;
            for (const auto &segment: polygon_segments) {
                // If there is an exact edge between these two points -- break as there won't be any intersections anymore
                if ((segment.first == segment_between_point.first && segment.second == segment_between_point.second) ||
                    (segment.second == segment_between_point.first && segment.first == segment_between_point.second)) {
                    break;
                }
                // If the segment starts in any of

                // TODO: check if this works properly for vertical and horizontal lines intersection
                // If there is a segment between two point -- they are not visible for each other
                auto intersection = segment_segment_intersection(segment_between_point, segment);
                if (intersection.first >= std::min(segment.first.first, segment.second.first) &&
                    intersection.first <= std::max(segment.first.first, segment.second.first)) {

                }
            }
        }
    }

}