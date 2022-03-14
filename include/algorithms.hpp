#ifndef MAP_TO_GRAPH_ALGORITHMS_HPP
#define MAP_TO_GRAPH_ALGORITHMS_HPP

#include <vector>
#include "Graph.hpp"

using vpdd = std::vector<std::pair<double, double>>;


/*!
 * Calculate the sweeping path to cover the graph g
 * NOTE: If the area to cover is not a convex polygon, the algorithm will produce not a full coverage path
 * @param g Graph that needs to be covered
 * @param start_up Whether the first column should be covered by movement up or down
 * @return A sequence of points (longitude, latitude) that need to be visited by drone
 */
vpdd sweeping(const Graph &g, bool start_up=false);


#endif //MAP_TO_GRAPH_ALGORITHMS_HPP
