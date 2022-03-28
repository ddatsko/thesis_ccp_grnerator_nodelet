//
// Created by mrs on 28.03.22.
//

#include "mstp_solver/TargetSet.h"
#include "algorithms.hpp"
#include "Graph.hpp"

TargetSet::TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step, const EnergyCalculator &energy_calculator,
                     const std::vector<double> &rotation_angles) : polygon(polygon),
                                                                   energy_calculator(energy_calculator) {
    for (auto angle: rotation_angles) {
        auto graph_from_polygon = Graph(polygon, angle, sweeping_step);
        for (int i = 0; i < 2; i++) {
            auto sweeping_path = sweeping(graph_from_polygon, static_cast<bool>(i));
            double path_energy = energy_calculator.calculate_path_energy_consumption(sweeping_path);
            targets.push_back(Target{static_cast<bool>(i),
                                     angle,
                                     path_energy,
                                     sweeping_path[0],
                                     sweeping_path[sweeping_path.size() - 1],
                                     index});
        }
    }
}
