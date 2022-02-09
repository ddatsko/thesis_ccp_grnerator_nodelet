#ifndef MAP_TO_GRAPH_UTILS_HPP
#define MAP_TO_GRAPH_UTILS_HPP

#include <tuple>

typedef std::tuple<double, double, double> hom_t;

hom_t cross_product(const hom_t &a, const hom_t &b);


#endif //MAP_TO_GRAPH_UTILS_HPP
