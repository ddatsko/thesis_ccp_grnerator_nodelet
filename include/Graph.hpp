//
// Created by mrs on 24.01.22.
//

#ifndef MAP_TO_GRAPH_GRAPH_HPP
#define MAP_TO_GRAPH_GRAPH_HPP

#include <cstddef>
#include <vector>

/*!
 * Custom graph class for representing the graph in such a format:
 * - coordinates of the left bottom corner
 * - steps between each cell
 * - bool array of values, determining the presence of a vertex in the cell
 */
class Graph {
private:
    double smallest_x, smallest_y;
    double step;
    size_t height, width;
    std::vector<bool> vertices_matrix;
public:
    Graph(double smallest_x, double biggest_x, double smallest_y, double biggest_y, double step);
    bool operator()(size_t row, size_t col) const;
    void add_vertex(int row, int col);
    void remove_vertex(int row, int col);
    std::pair<double, double> get_origin() const;
    [[nodiscard]] size_t get_width() const;
    [[nodiscard]] size_t get_height() const;
    [[nodiscard]] double get_step() const;
};


#endif //MAP_TO_GRAPH_GRAPH_HPP
