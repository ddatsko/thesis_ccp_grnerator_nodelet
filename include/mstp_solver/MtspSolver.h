//
// Created by mrs on 28.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_MTSPSOLVER_H
#define THESIS_TRAJECTORY_GENERATOR_MTSPSOLVER_H

#include "SolverConfig.h"
#include "TargetSet.h"
#include <vector>
#include "MapPolygon.hpp"
#include "Target.h"

using solution_t = std::vector<std::vector<Target>>;

class MtspSolver {
private:
    std::vector<TargetSet> m_target_sets;
    const SolverConfig m_config;
    const EnergyCalculator m_energy_calculator;

    solution_t greedy_random();

    double get_path_cost(const std::vector<Target> &path);

public:
    MtspSolver(SolverConfig  config,
               const std::vector<MapPolygon> &decomposed_polygons,
               const EnergyCalculator &energy_calculator);

    solution_t solve();
};


#endif //THESIS_TRAJECTORY_GENERATOR_MTSPSOLVER_H
