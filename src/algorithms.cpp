#include "algorithms.hpp"
#include <algorithm>
#include <utils.hpp>
#include "custom_types.hpp"
#include <iostream>
#include <cmath>


namespace {
    constexpr double POINT_EPS = 1e-5;

    double vertical_line_segment_intersection(const segment_t &segment, double x) {
        return (x - segment.first.first) * (segment.second.second - segment.first.second) /
               (segment.second.first - segment.first.first) + segment.first.second;
    }


    /*!
     * Add new polygon detected by trapezoidal decomposition to the existing set of polygons
     * Merges the polygon with an existing one if new polygon's left edge is
     * one of the edges of the existing one and merge_to_boustrophedon is set to true
     * @warning Because of the whole algorithm specifications, here the following assumption is made:
     *    Each polygon points start with {lower(leftmost points), upper(leftmost point)}.
     *    So, points are in clockwise order, starting with lower of left point (there are at leads 2 of them)
     * @param polygons Set of polygons
     * @param polygon New polygon to be added
     * @param decomposition_type Type of the decomposition according to which he polygon will be added
     */
    void add_polygon(std::vector<MapPolygon> &polygons, const MapPolygon &new_polygon,
                     decomposition_type_t decomposition_type) {
        // Just add polygon if there is no need to merge them
        if (decomposition_type == TRAPEZOIDAL_DECOMPOSITION) {
            polygons.push_back(new_polygon);
            return;
        }

        bool inserted = false;

        if (new_polygon.fly_zone_polygon_points.size() <= 2) {
            throw polygon_decomposition_error("Merging an empty polygon with another one");
        }

        // Edge of the new polygon that can be merged with some polygon to the left from it
        segment_t merging_edge = {new_polygon.fly_zone_polygon_points[0], new_polygon.fly_zone_polygon_points[1]};

        // Try to merge with each of existing polygons and break if a proper one is found
        for (auto &p: polygons) {
            if (p.fly_zone_polygon_points.size() <= 2) {
                // Should never get here, but in case...
                throw polygon_decomposition_error("Merging polygon with an empty polygon...");
            }
            auto rightmost_edge = p.rightmost_edge();
//            std::cout << "Rightmost edge: " << rightmost_edge.first.first << ", " << rightmost_edge.first.second << ";   " << rightmost_edge.second.first << ", " << rightmost_edge.second.second << std::endl;
            // If merging edge matches
            if (isclose(rightmost_edge.second.first, merging_edge.first.first, POINT_EPS) &&
              isclose(rightmost_edge.second.second, merging_edge.first.second, POINT_EPS) &&
              isclose(rightmost_edge.first.first, merging_edge.second.first, POINT_EPS) &&
              isclose(rightmost_edge.first.second, merging_edge.second.second, POINT_EPS)) {
//                std::cout << "Inserting" << std::endl;
                auto p_old = p;

                // Insert edge of the new polygon between merging edge ends excluding duplicates
                p.fly_zone_polygon_points.insert(
                        std::find(p.fly_zone_polygon_points.begin(), p.fly_zone_polygon_points.end(),
                                  rightmost_edge.second),
                        new_polygon.fly_zone_polygon_points.begin() + static_cast<long>(2),
                        new_polygon.fly_zone_polygon_points.end() - 1);

                if (decomposition_type==BOUSTROPHEDON_WITH_CONVEX_POLYGONS && !polygon_convex(p.fly_zone_polygon_points)) {
                    p = p_old;
                    polygons.push_back(new_polygon);
                }
                inserted = true;
                break;
            }
        }
        // If an appropriate polygon for merging was not found - just add the polygon to list
        if (!inserted) {
            polygons.push_back(new_polygon);
        }

    }
}


MapPolygon polygon_from_2_segments(segment_t s1, segment_t s2, double x) {
    MapPolygon polygon;
    if (s1.first.second > s2.first.second || s1.second.second > s2.second.second) {
        std::swap(s1, s2);
    }
    // Triangle with one point in the left
    if (isclose(s1.first, s2.first, POINT_EPS)) {
        polygon.fly_zone_polygon_points.insert(polygon.fly_zone_polygon_points.end(),
                                               {
                                                       s1.first,
                                                       {x, vertical_line_segment_intersection(s2, x)},
                                                       {x, vertical_line_segment_intersection(s1, x)},
                                                       s1.first
                                               });
        return polygon;
    }

    // Triangle with one point which is the same as the point of segments intersection at the right
    if (isclose(s2.second.first, x, POINT_EPS) && isclose(s2.second, s1.second, POINT_EPS)) {
        polygon.fly_zone_polygon_points.insert(polygon.fly_zone_polygon_points.end(),
                                               {
                                                       s1.first,
                                                       s2.first,
                                                       s1.second,
                                                       s1.first
                                               });
        return polygon;
    }

    polygon.fly_zone_polygon_points.insert(polygon.fly_zone_polygon_points.end(), {
            s1.first,
            s2.first,
            {x, vertical_line_segment_intersection(s2, x)},
            {x, vertical_line_segment_intersection(s1, x)},
            s1.first
    });
    return polygon;
}


vpdd sweeping(const MapPolygon &polygon, double angle, double sweeping_step, bool start_up) {
    auto rotated_polygon = polygon.rotated(angle);
    vpdd res_path;
    double leftmost_border = std::numeric_limits<double>::max();
    for (const auto &p: rotated_polygon.get_all_points()) {
        leftmost_border = std::min(leftmost_border, p.first);
    }
    double current_x = leftmost_border + sweeping_step / 2;
    bool current_direction_up = start_up;

    bool last_one = false;
    while (true) {
        std::set<double> intersection_ys;
        for (const auto &segment: rotated_polygon.get_all_segments()) {
            hom_t vertical_line = {1, 0, -current_x};
            if ((segment.first.first < current_x && segment.second.first < current_x) ||
                    (segment.first.first > current_x && segment.second.first > current_x)) {
                continue;
            }
            auto intersection = segment_line_intersection(segment.first, segment.second, vertical_line);
            intersection_ys.insert(intersection.second);
        }
        if (intersection_ys.size() <= 1) {
            if (last_one) {
                break;
            }
            current_x -= sweeping_step / 2;
            last_one = true;
            continue;
        }
        double lower_y = *intersection_ys.begin();
        double upper_y = *(++intersection_ys.begin());
        if (upper_y - lower_y < sweeping_step) {
            res_path.emplace_back(current_x, (upper_y + lower_y) / 2);
        } else {
            if (current_direction_up) {
                res_path.emplace_back(current_x, lower_y + sweeping_step / 2);
                res_path.emplace_back(current_x, upper_y - sweeping_step / 2);
                current_direction_up = false;
            } else {
                res_path.emplace_back(current_x, upper_y - sweeping_step / 2);
                res_path.emplace_back(current_x, lower_y + sweeping_step / 2);
                current_direction_up = true;
            }
        }
        current_x += sweeping_step;
    }

    //TODO: find out what to do in the situation when the polygon is so thin, that we cannot do any sweeping. For now -- just add one point
    if (res_path.empty()) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        for (const auto&[x, y]: polygon.fly_zone_polygon_points) {
            sum_x += x;
            sum_y += y;
        }
        res_path.push_back({sum_x / polygon.fly_zone_polygon_points.size(), sum_y / polygon.fly_zone_polygon_points.size()});
    }

    std::for_each(res_path.begin(), res_path.end(), [angle](auto &p){p = rotate_point(p, -angle);});

    return res_path;
}

// NOTE: this function assumes that all the non-fly zones are located inside the fly-zone area,
// and no non-fly zones overlap
std::vector<MapPolygon> trapezoidal_decomposition(const MapPolygon &polygon, decomposition_type_t decomposition_type) {
    // Generate a set of all the points of both fly-zone and non-fly-zone polygons
    const auto points = polygon.get_all_points();
    std::vector<MapPolygon> res;

    // Segments that are currently crossed by the sweeping line
    std::vector<segment_t> current_segments;

    // Iterate through all the "interesting" points
    for (const auto &p: points) {
        const double x = p.first, y = p.second;
        auto neighbors = polygon.point_neighbors(p);

        segment_t lower_segment = {p, (std::sin(get_segment_rotation({p, neighbors.first})) < std::sin(get_segment_rotation({p, neighbors.second})) ? neighbors.first
                                                                                        : neighbors.second)};
        segment_t upper_segment = {p, (std::sin(get_segment_rotation({p, neighbors.first})) < std::sin(get_segment_rotation({p, neighbors.second}))  ? neighbors.second
                                                                                        : neighbors.first)};

        // Two segments starting in this point are moving to the right.
        // Case of area is split by a no-fly zone inside (e.g. island)
        if (neighbors.first.first >= x && neighbors.second.first >= x) {
//            std::cout << "DIVERGE" << std::endl;
            bool found_space = false;
            for (size_t i = 0; i < current_segments.size(); ++i) {
                if (vertical_line_segment_intersection(current_segments[i], x) > y) {
                    found_space = true;
                    // If the point is inside the current fly-zone - add new trapezoid and cut other segments
                    if (i % 2 == 1) {
                        MapPolygon polygon_chunk = polygon_from_2_segments(current_segments[i - 1], current_segments[i],
                                                                           x);
                        add_polygon(res, polygon_chunk, decomposition_type);
                        current_segments[i - 1].first = {x, vertical_line_segment_intersection(current_segments[i - 1],
                                                                                               x)};
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


        // Case when two segments converge. Thus, two new trapezoids must be added and two edges deleted
        if (neighbors.first.first <= x && neighbors.second.first <= x) {
//            std::cout << "CONVERGE" << std::endl;
            bool found_space = false;
            for (size_t i = 0; i < current_segments.size(); ++i) {
                if (isclose(current_segments[i].second, p, POINT_EPS)) {
//                    std::cout << "CURRENT SGEMENTS: " << std::endl;
//                    for (const auto &s: current_segments) {
//                        std::cout << s.first.first << ", " << s.first.second << ";   " << s.second.first << ", " << s.second.second << std::endl;
//                    }
//                    std::cout << "==========================" << std::endl;

                    found_space = true;
                    // last two converging segments
                    if (i % 2 == 0) {
//                        std::cout << "FLY-ZONE CONVERGING" << std::endl;
                        add_polygon(res, polygon_from_2_segments(current_segments[i], current_segments[i + 1], x),
                                    decomposition_type);
                        current_segments.erase(current_segments.begin() + static_cast<long>(i));
                        current_segments.erase(current_segments.begin() + static_cast<long>(i));
                        break;
                    }

                    add_polygon(res, polygon_from_2_segments(current_segments[i], current_segments[i - 1], x),
                                decomposition_type);
                    add_polygon(res, polygon_from_2_segments(current_segments[i + 1], current_segments[i + 2], x),
                                decomposition_type);
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
//        std::cout << "NORMAL" << std::endl;
        // The third case: two neighbors are on different sides of the line
        if (neighbors.second.first < neighbors.first.first) {
            std::swap(neighbors.first, neighbors.second);
        }

        bool segment_found = false;
        for (size_t i = 0; i < current_segments.size(); ++i) {
            if (isclose(current_segments[i].second, p, POINT_EPS)) {
                segment_found = true;
                if (i % 2 == 0) {
                    // The gap between segments[i] and segments[i + 1] is the fly-zone
                    add_polygon(res, polygon_from_2_segments(current_segments[i], current_segments[i + 1], x),
                                decomposition_type);
                    current_segments[i + 1].first = {x, vertical_line_segment_intersection(current_segments[i + 1], x)};
                } else {
                    // The gap between segments[i] and segments[i - 1] is the fly-zone
                    add_polygon(res, polygon_from_2_segments(current_segments[i - 1], current_segments[i], x),
                                decomposition_type);
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
