//
// Created by mrs on 24.01.22.
//

#ifndef MAP_TO_GRAPH_GRAPH_HPP
#define MAP_TO_GRAPH_GRAPH_HPP

#include <cstddef>
#include <vector>
#include "MapPolygon.hpp"

/*!
 * Custom graph class for representing the graph in such a format:
 * - coordinates of the left bottom corner
 * - steps between each cell
 * - bool array of values, determining the presence of a vertex in the cell
 */
class Graph {
private:
    double m_smallest_x, m_smallest_y;
    double m_rotation_angle;
    double m_step;
    size_t m_height, m_width;
    std::vector<bool> m_vertices_matrix;
public:
    /*!
     * @param map_polygon MapPolygon to construct the graph from
     * @param rotation_angle Angle of MapPolygon rotation during the grid construction
     * @param step Step of grid in the same units as MapPolygon points [change of latitude/longitude]
     */
    Graph(const MapPolygon &map_polygon, double rotation_angle, double step);
    bool operator()(size_t row, size_t col) const;
    void add_vertex(int row, int col);
    point_t get_point_coords(size_t row, size_t col) const;

    [[nodiscard]] size_t get_width() const;
    [[nodiscard]] size_t get_height() const;
    [[nodiscard]] double get_step() const;
};


#endif //MAP_TO_GRAPH_GRAPH_HPP
