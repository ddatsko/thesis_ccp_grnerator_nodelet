#include "algorithms.hpp"
#include <iostream>
#include <queue>
#include <iomanip>
#include <stack>
#include <set>
#include <algorithm>


using segment_t = std::pair<point_t, point_t>;

namespace {
    double vertical_line_segment_intersection(const segment_t &segment, double x) {
        return (x - segment.first.first) * (segment.second.second - segment.first.second) /
               (segment.second.first - segment.first.first) + segment.first.second;
    }

    /*!
     * Create a map polygon from 2 segments and a vertical line on the right
     * @note Each vertical line must go through each fo 2 segments. Otherwise the result is unfeasible
     * @param s1 Segment 1
     * @param s2 Segment 2
     * @param x x coordinate of vertical line
     * @return MapPolygon created
     */
    MapPolygon polygon_from_2_segments(segment_t s1, segment_t s2, double x) {
        MapPolygon polygon;
        // Triangle with one point in the left
        if (s1.first == s2.first) {
            polygon.fly_zone_polygon_points.insert(polygon.fly_zone_polygon_points.end(),
                                                   {
                                                           s1.first,
                                                           {x, vertical_line_segment_intersection(s1, x)},
                                                           {x, vertical_line_segment_intersection(s2, x)},
                                                           s1.first
                                                   });
            return polygon;
        }

        // Triangle with one point which is the same as the point of segments intersection at the right
        if (s2.second.first == x && s2.second == s1.second) {
            polygon.fly_zone_polygon_points.insert(polygon.fly_zone_polygon_points.end(),
                                                   {
                                                           s1.first,
                                                           s1.second,
                                                           s2.first,
                                                           s1.first
                                                   });
            return polygon;
        }

        polygon.fly_zone_polygon_points.insert(polygon.fly_zone_polygon_points.end(), {
            s1.first,
            {x, vertical_line_segment_intersection(s1, x)},
            {x, vertical_line_segment_intersection(s2, x)},
            s2.first,
            s1.first
        });
        return polygon;
    }
}


vpdd sweeping(const Graph &g, bool start_up) {
    const size_t w = g.get_width(), h = g.get_height();
    vpdd res_path;

    int dy = start_up ? -1 : 1;
    size_t row = start_up ? (h - 1) : 0;
    size_t col = 0;
    while (col < w) {
        if (g(row, col)) {
            res_path.push_back(g.get_point_coords(row, col));
//            res_path.push_back({static_cast<double>(row), static_cast<double>(col)});
        }
        if ((row == 0 && dy == -1) || (row + dy == h)) {
            col++;
            dy *= -1;
            row = (dy == -1) ? (h - 1) : 0;
        } else {
            row += dy;
        }

    }
    return res_path;
}


// NOTE: this function assumes that all the non-fly zones are located inside the fly-zone area,
// and no non-fly zones overlap
std::vector<MapPolygon> trapezoidal_decomposition(const MapPolygon &polygon) {
    // Generate a set of all the points of both fly-zone and non-fly-zone polygons
    const auto points = polygon.get_all_points();
    std::vector<MapPolygon> res;

    // Segments that are currently crossed by the sweeping line
    std::vector<segment_t> current_segments;

    // Iterate through all the "interesting" points
    for (const auto &p: points) {
        const double x = p.first, y = p.second;
        auto neighbors = polygon.point_neighbors(p);

        segment_t lower_segment = {p, (neighbors.first.second < neighbors.second.second ? neighbors.first
                                                                                   : neighbors.second)};
        segment_t upper_segment = {p, (neighbors.first.second < neighbors.second.second ? neighbors.second
                                                                                   : neighbors.first)};

        // Two segments starting in this point are moving to the right.
        // Case of area is split by a no-fly zone inside (e.g. island)
        if (neighbors.first.first >= x && neighbors.second.first >= x) {
            bool found_space = false;
            for (size_t i = 0; i < current_segments.size(); ++i) {
                if (vertical_line_segment_intersection(current_segments[i], x) > y) {
                    found_space = true;
                    // If the point is inside the current fly-zone - add new trapezoid and cut other segments
                    if (i % 2 == 1) {
                        MapPolygon polygon_chunk = polygon_from_2_segments(current_segments[i - 1], current_segments[i], x);
                        res.push_back(polygon_chunk);
                        current_segments[i - 1].first = {x, vertical_line_segment_intersection(current_segments[i - 1], x)};
                        current_segments[i].first = {x, vertical_line_segment_intersection(current_segments[i], x)};
                    }
                    current_segments.insert(current_segments.begin() + static_cast<long>(i), upper_segment);
                    current_segments.insert(current_segments.begin() + static_cast<long>(i), lower_segment);
                    break;
                }
            }
            if (!found_space) {
                // Insert 2 new segments keeping the order that the segment with lower y is located lower
                current_segments.push_back(lower_segment);
                current_segments.push_back(upper_segment);
                continue;
            }
            continue;
        }


        // TODO: rewrite this without code copy-paste
        // Case when two segments converge. Thus, two new trapezoids must be added and two edges deleted
        if (neighbors.first.first <= x && neighbors.second.first <= x) {
            bool found_space = false;
            for (size_t i = 0; i < current_segments.size(); ++i) {
                if (current_segments[i].second == p) {
                    found_space = true;
                    // last two converging segments
                    if (i % 2 == 0) {
                        res.push_back(polygon_from_2_segments(current_segments[i], current_segments[i + 1], x));
                        current_segments.erase(current_segments.begin() + static_cast<long>(i));
                        current_segments.erase(current_segments.begin() + static_cast<long>(i));
                        break;
                    }

                    if (i % 2 != 1) {
                        throw polygon_decomposition_error("Non-fly zone converging outside the fly-zone polygon");
                    }
                    res.push_back(polygon_from_2_segments(current_segments[i], current_segments[i - 1], x));
                    res.push_back(polygon_from_2_segments(current_segments[i + 1], current_segments[i + 2], x));
                    current_segments[i - 1].first = {x, vertical_line_segment_intersection(current_segments[i - 1], x)};
                    current_segments[i + 2].first = {x, vertical_line_segment_intersection(current_segments[i + 2], x)};
                    current_segments.erase(current_segments.begin() + static_cast<long>(i));
                    current_segments.erase(current_segments.begin() + static_cast<long>(i));
                    break;
                }
            }
            if (!found_space) {
                throw polygon_decomposition_error("Non-fly zone converging outside the fly-zone polygon");
            }
            continue;
        }

        // The third case: two neighbors are on different sides of the line
        if (neighbors.second.first < neighbors.first.first) {
            std::swap(neighbors.first, neighbors.second);
        }

        bool segment_found = false;
        for (size_t i = 0; i < current_segments.size(); ++i) {
            if (current_segments[i].second == p) {
                segment_found = true;
                if (i % 2 == 0) {
                    // The gap between segments[i] and segments[i + 1] is the fly-zone
                    res.push_back(polygon_from_2_segments(current_segments[i], current_segments[i + 1], x));
                    current_segments[i + 1].first = {x, vertical_line_segment_intersection(current_segments[i + 1], x)};
                } else {
                    // The gap between segments[i] and segments[i - 1] is the fly-zone
                    res.push_back(polygon_from_2_segments(current_segments[i], current_segments[i - 1], x));
                    current_segments[i - 1].first = {x, vertical_line_segment_intersection(current_segments[i - 1], x)};
                }
                current_segments[i] = {p, neighbors.second};
                break;
            }
        }

        if (!segment_found) {
            throw polygon_decomposition_error("Checkpoint does not belong to any polygon..");
        }
    }
    return res;
}