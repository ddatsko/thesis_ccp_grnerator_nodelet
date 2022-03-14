#include "algorithms.hpp"
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <queue>
#include <iomanip>
#include <stack>


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
