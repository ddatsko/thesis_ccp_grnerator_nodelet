#include "algorithms.hpp"
#include <algorithm>
#include <utils.hpp>
#include "custom_types.hpp"


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
        if (s1.first.second > s2.first.second) {
            std::swap(s1, s2);
        }
        // Triangle with one point in the left
        if (s1.first == s2.first) {
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
        if (s2.second.first == x && s2.second == s1.second) {
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

    /*!
     * Add new polygon detected by trapezoidal decomposition to the existing set of polygons
     * Merges the polygon with an existing one if new polygon's left edge is
     * one of the edges of the existing one and merge_to_boustrophedon is set to true
     * @warning Because of the whole algorithm specifications, here the following assumption is made:
     *    Each polygon points start with {lower(leftmost points), upper(leftmost point)}.
     *    So, points are in clockwise order, starting with lower of left point (there are at leads 2 of them)
     * @param polygons Set of polygons
     * @param polygon New polygon to be added
     */
    void add_polygon(std::vector<MapPolygon> &polygons, const MapPolygon &new_polygon,
                     bool merge_to_boustrophedon = true) {
        // Just add polygon if there is no need to merge them
        if (!merge_to_boustrophedon) {
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

            // If merging edge matches
            if (rightmost_edge.second == merging_edge.first &&
                rightmost_edge.first == merging_edge.second) {

                // Insert edge of the new polygon between merging edge ends excluding duplicates
                p.fly_zone_polygon_points.insert(
                        std::find(p.fly_zone_polygon_points.begin(), p.fly_zone_polygon_points.end(),
                                  rightmost_edge.second),
                        new_polygon.fly_zone_polygon_points.begin() + static_cast<long>(2),
                        new_polygon.fly_zone_polygon_points.end() - 1);
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


vpdd sweeping(const MapPolygon &polygon, double angle, double sweeping_step, bool start_up) {
    auto rotated_polygon = polygon.rotated(angle);
    vpdd res_path;
    double leftmost_border = std::numeric_limits<double>::max();
    for (const auto &p: rotated_polygon.get_all_points()) {
        leftmost_border = std::min(leftmost_border, p.first);
    }
    double current_x = leftmost_border + sweeping_step / 2;
    bool current_direction_up = start_up;

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
            break;
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

    //TODO: find out what ro do in the situation when the polygon is so thin, that we cannot do any sweeping. For now -- just add one
    if (res_path.empty()) {
        res_path.push_back(rotated_polygon.fly_zone_polygon_points[0]);
    }

    std::for_each(res_path.begin(), res_path.end(), [angle](auto &p){p = rotate_point(p, -angle);});

    return res_path;
}


// NOTE: this function assumes that all the non-fly zones are located inside the fly-zone area,
// and no non-fly zones overlap
std::vector<MapPolygon> trapezoidal_decomposition(const MapPolygon &polygon, bool merge_to_boustrophedon) {
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
                        MapPolygon polygon_chunk = polygon_from_2_segments(current_segments[i - 1], current_segments[i],
                                                                           x);
                        add_polygon(res, polygon_chunk, merge_to_boustrophedon);
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


        // TODO: rewrite this without code copy-paste
        // Case when two segments converge. Thus, two new trapezoids must be added and two edges deleted
        if (neighbors.first.first <= x && neighbors.second.first <= x) {
            bool found_space = false;
            for (size_t i = 0; i < current_segments.size(); ++i) {
                if (current_segments[i].second == p) {
                    found_space = true;
                    // last two converging segments
                    if (i % 2 == 0) {
                        add_polygon(res, polygon_from_2_segments(current_segments[i], current_segments[i + 1], x),
                                    merge_to_boustrophedon);
                        current_segments.erase(current_segments.begin() + static_cast<long>(i));
                        current_segments.erase(current_segments.begin() + static_cast<long>(i));
                        break;
                    }

                    add_polygon(res, polygon_from_2_segments(current_segments[i], current_segments[i - 1], x),
                                merge_to_boustrophedon);
                    add_polygon(res, polygon_from_2_segments(current_segments[i + 1], current_segments[i + 2], x),
                                merge_to_boustrophedon);
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
                    add_polygon(res, polygon_from_2_segments(current_segments[i], current_segments[i + 1], x),
                                merge_to_boustrophedon);
                    current_segments[i + 1].first = {x, vertical_line_segment_intersection(current_segments[i + 1], x)};
                } else {
                    // The gap between segments[i] and segments[i - 1] is the fly-zone
                    add_polygon(res, polygon_from_2_segments(current_segments[i], current_segments[i - 1], x),
                                merge_to_boustrophedon);
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
