#ifndef MAP_TO_GRAPH_ALGORITHMS_HPP
#define MAP_TO_GRAPH_ALGORITHMS_HPP

#include <vector>
#include "Graph.hpp"
#include "MapPolygon.hpp"

using vpdd = std::vector<std::pair<double, double>>;

struct polygon_decomposition_error: public std::runtime_error {
    using runtime_error::runtime_error;
};

/*!
 * Calculate the sweeping path to cover the graph g
 * NOTE: If the area to cover is not a convex polygon, the algorithm will produce not a full coverage path
 * @param g Graph that needs to be covered
 * @param start_up Whether the first column should be covered by movement up or down
 * @return A sequence of points (longitude, latitude) that need to be visited by drone
 */
vpdd sweeping(const Graph &g, bool start_up=false);

/*!
 * Decompose the MapPolygon into smaller MapPolygons using trapezoidal decomposition algorithm
 * @param polygon MapPolygon to be decomposed
 * @param merge_to_boustrophedon If true, trapezoids will be merged if they have a common edge (boustrophedon decomposition)
 * @return Vector of convex polygons with no no-fly-zones and such that
 * union(result) = original_polygon
 * intersection(result) = {}
 */
std::vector<MapPolygon> trapezoidal_decomposition(const MapPolygon &polygon,  bool merge_to_boustrophedon=false);


#endif //MAP_TO_GRAPH_ALGORITHMS_HPP
