//
// Created by mrs on 24.01.22.
//
#include <iostream>
#include "Graph.hpp"

Graph::Graph(double smallest_x, double biggest_x, double smallest_y, double biggest_y, double step) :
        smallest_x(smallest_x), smallest_y(smallest_y), step(step) {

    height = (size_t)((biggest_y - smallest_y) / step) + 2;
    width = (size_t)((biggest_x - smallest_x) / step) + 2;
    vertices_matrix = std::vector<bool>(width * height, false);
}

bool Graph::operator()(size_t row, size_t col) const {
    return vertices_matrix[row * width + col];
}

void Graph::add_vertex(int row, int col) {
    vertices_matrix[row * width + col] = true;
}

void Graph::remove_vertex(int row, int col) {
    vertices_matrix[row * width + col] = false;
}

size_t Graph::get_height() const {
    return height;
}

size_t Graph::get_width() const {
    return width;
}

double Graph::get_step() const {
    return step;
}

std::pair<double, double> Graph::get_origin() const {
    return {smallest_y, smallest_x};
}
