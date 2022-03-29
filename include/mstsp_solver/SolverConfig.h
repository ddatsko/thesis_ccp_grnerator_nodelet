//
// Created by mrs on 28.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_SOLVERCONFIG_H
#define THESIS_TRAJECTORY_GENERATOR_SOLVERCONFIG_H

#include "utils.hpp"
#include <vector>

namespace mstsp_solver {
    struct SolverConfig {
        std::vector<double> rotation_angles;
        double sweeping_step;
        point_t starting_point;
        size_t n_uavs;

    };
}
#endif //THESIS_TRAJECTORY_GENERATOR_SOLVERCONFIG_H
