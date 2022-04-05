//
// Created by mrs on 28.03.22.
//

#include "mstsp_solver/TargetSet.h"
#include "algorithms.hpp"
#include "Graph.hpp"
#include <iostream>
#include <iomanip>

namespace mstsp_solver {

    TargetSet::TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step,
                         const EnergyCalculator &energy_calculator,
                         const std::vector<double> &rotation_angles) : index(index), polygon(polygon),
                                                                       energy_calculator(energy_calculator),
                                                                       sweeping_step(sweeping_step) {
        add_rotation_angles(rotation_angles);
    }


    TargetSet::TargetSet(size_t index,
                         const MapPolygon &polygon,
                         double sweeping_step,
                         const EnergyCalculator &energy_calculator,
                         size_t number_of_edges_rotations) : index(index), polygon(polygon),
                                                             energy_calculator(energy_calculator),
                                                             sweeping_step(sweeping_step) {
        add_rotation_angles(polygon.get_n_longest_edges_rotation_angles(number_of_edges_rotations));
    }


    void TargetSet::add_rotation_angles(const std::vector<double> &rotation_angles) {
        for (auto angle: rotation_angles) {
            for (int i = 0; i < 2; i++) {
                auto sweeping_path = sweeping(polygon, angle, sweeping_step, static_cast<bool>(i));
                double path_energy = energy_calculator.calculate_path_energy_consumption(sweeping_path);

                targets.push_back(Target{static_cast<bool>(i),
                                         angle,
                                         path_energy,
                                         sweeping_path[0],
                                         sweeping_path[sweeping_path.size() - 1],
                                         index,
                                         targets.size()});
            }
        }
    }
}