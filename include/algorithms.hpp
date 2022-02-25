#ifndef MAP_TO_GRAPH_ALGORITHMS_HPP
#define MAP_TO_GRAPH_ALGORITHMS_HPP

#include <vector>
#include "Graph.hpp"

/*!
 * Wavefront algorithm for one drone
 * @param start_lat Start latitude
 * @param start_long Start longitude
 * @param g Graph to use the algorithm on
 * @return Empty vector in case of bad arguments (e.g. start position is not in the graph),
 *         A sequence of (latitude, longitude) locations to be visited by the drone
 */
std::vector<std::pair<double, double>> wavefront(double start_lat, double start_long, const Graph &g);



#endif //MAP_TO_GRAPH_ALGORITHMS_HPP
