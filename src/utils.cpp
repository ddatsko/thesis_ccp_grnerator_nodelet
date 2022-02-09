#include "utils.hpp"

hom_t cross_product(const hom_t &a, const hom_t &b) {
    return {std::get<1>(a) * std::get<2>(b) - std::get<2>(a) * std::get<1>(b),
            std::get<2>(a) * std::get<0>(b) - std::get<0>(a) * std::get<2>(b),
            std::get<0>(a) * std::get<1>(b) - std::get<1>(a) * std::get<0>(b)};
}
