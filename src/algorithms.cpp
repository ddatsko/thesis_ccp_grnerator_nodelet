#include "algorithms.hpp"
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <queue>
#include <iomanip>
#include <stack>

struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator () (std::pair<T1, T2> const &v) const
    {
        return std::hash<T1>()(v.first) * std::hash<T2>()(v.second);
    }
};

const int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
const int dy[8] = {0, -1, 0, 1, 1, -1, 1, -1};


void dfs(size_t row,
         size_t col,
         std::unordered_set<std::pair<size_t, size_t>, pair_hash> &visited,
         std::vector<std::pair<double, double>> &path,
         const Graph &g,
         std::unordered_map<std::pair<size_t, size_t>, int, pair_hash> levels);


std::vector<std::pair<double, double>> wavefront(double start_lat, double start_long, const Graph &g) {
    std::vector<std::pair<double, double>> path;
    auto graph_origin = g.get_origin();
    double cell_step = g.get_step();

    // Check if the starting point is inside the desired region
    if (start_lat < graph_origin.first or start_long < graph_origin.second or
            (size_t)((start_lat - graph_origin.first) / cell_step) >= g.get_height() or
            (size_t)((start_long - graph_origin.second) / cell_step) >= g.get_width()) {
        std::cerr << "Error in the algorithm: start point is not inside the polygon..." << std::endl;
        return path;
    }

    // Calculate the start cell by start coordinates
    auto start_cell_col = (size_t)((start_long - graph_origin.second) / cell_step);
    auto start_cell_row = (size_t)((start_lat - graph_origin.first) / cell_step);

    // Run BFS as a first step of wavefront algorithm to fii in the "levels"
    std::unordered_set<std::pair<size_t, size_t>, pair_hash> visited;
    visited.insert({start_cell_row, start_cell_col});

    std::unordered_map<std::pair<size_t, size_t>, int, pair_hash> levels;
    levels[{start_cell_row, start_cell_col}] = 1;
    std::queue<std::pair<std::pair<size_t, size_t>, int>> bfs_queue;
    bfs_queue.push({{start_cell_row, start_cell_col}, 1});

    while (!bfs_queue.empty()) {
        int level = bfs_queue.front().second;
        auto cell = bfs_queue.front().first;
        bfs_queue.pop();

        // Go through each neighbour checking if we haven't left the desired area and if that cell hasn't been visited before
        for (int i = 0; i < 4; i++) {
            if ((cell.first == 0 and dx[i] == -1) or (cell.second == 0 and dy[i] == -1)) {
              continue;
            }
            size_t new_x = cell.first + dx[i];
            size_t new_y = cell.second + dy[i];

            if (visited.find({new_x, new_y}) == visited.end() and new_x < g.get_height() and new_y < g.get_width() and g(new_x, new_y)) {
                visited.insert({new_x, new_y});
                levels[{new_x, new_y}] = level + 1;
                bfs_queue.push({{new_x, new_y}, level + 1});
            }
        }
    }

    // Run the DFS as the second stage of the algorithm tp find out the possible path for the drone
    visited.clear();
    path.emplace_back(start_lat, start_long);
    visited.insert({start_cell_row, start_cell_col});

    dfs(start_cell_row, start_cell_col, visited, path, g, levels);

    return path;
}

void dfs(size_t row,
         size_t col,
         std::unordered_set<std::pair<size_t, size_t>, pair_hash> &visited,
         std::vector<std::pair<double, double>> &path,
         const Graph &g,
         std::unordered_map<std::pair<size_t, size_t>, int, pair_hash> levels) {

    visited.insert({row, col});
    std::pair<double, double> cell_coordinates = {static_cast<double>(row) * g.get_step() + g.get_origin().first, static_cast<double>(col) * g.get_step() + g.get_origin().second};
    path.emplace_back(cell_coordinates.first, cell_coordinates.second);

    bool visited_path = false;
    for (int i = 0; i < 8; i++) {
        if ((row == 0 and dy[i] == -1) or (col == 0 and dx[i] == -1)) {
          continue;
        }

        size_t new_row = row + dy[i];
        size_t new_col = col + dx[i];

        // Firstly, try to go to each of neighbouring cells if it is not visited and LEVEL IS GREATER
        if (visited.find({new_row, new_col}) == visited.end() and new_row < g.get_height()  and
                new_col < g.get_width() and g(new_row, new_col) and levels[{new_row, new_col}] >= levels[{new_row, new_col}]) {

            if (visited_path) {
                path.emplace_back(cell_coordinates.first, cell_coordinates.second);
            }
            dfs(new_row, new_col, visited, path, g, levels);
            visited_path = true;
        }
    }

    for (int i = 0; i < 8; i++) {
        if ((row == 0 and dy[i] == -1) or (col == 0 and dx[i] == -1)) {
          continue;
        }
        size_t new_row = row + dy[i];
        size_t new_col = col + dx[i];

        // Firstly, try to go to each of neighbouring cells if it is not visited and LEVEL IS GREATER
        if (visited.find({new_row, new_col}) == visited.end() and new_row < g.get_height() and
            new_col < g.get_width() and g(new_row, new_col) and levels[{new_row, new_col}] != 0) {

            if (visited_path) {
                path.emplace_back(cell_coordinates.first, cell_coordinates.second);
            }
            dfs(new_row, new_col, visited, path, g, levels);
            visited_path = true;
        }
    }
}
