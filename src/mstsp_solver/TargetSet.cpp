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
                                                                       energy_calculator(energy_calculator) {
//        std::cout << "ROtation angles in set: " << rotation_angles.size() << std::endl;
        for (auto angle: rotation_angles) {

//            std::cout << "Polygon for sweeping with step " << sweeping_step << std::endl;
            for (const auto &p: polygon.fly_zone_polygon_points) {
                std::cout << std::setprecision(15) << p.first << ", " << p.second << std::endl;
            }


            for (int i = 0; i < 2; i++) {
                auto sweeping_path = sweeping(polygon, angle, sweeping_step, static_cast<bool>(i));
                double path_energy = energy_calculator.calculate_path_energy_consumption(sweeping_path);
                if (sweeping_path.empty()) {
//                    std::cout << "Sweeping path empty";
                    continue;
                }
                targets.push_back(Target{static_cast<bool>(i),
                                         angle,
                                         path_energy,
                                         sweeping_path[0],
                                         sweeping_path[sweeping_path.size() - 1],
                                         index,
                                         targets.size()});
            }
        }
//        std::cout << "Targets in target_set " << targets.size() << std::endl;
    }
}