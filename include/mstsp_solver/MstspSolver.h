//
// Created by mrs on 28.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_MSTSPSOLVER_H
#define THESIS_TRAJECTORY_GENERATOR_MSTSPSOLVER_H

#include "SolverConfig.h"
#include "TargetSet.h"
#include <vector>
#include "MapPolygon.hpp"
#include "Target.h"

namespace mstsp_solver {

    using solution_t = std::vector<std::vector<Target>>;

    class MstspSolver {
    private:
        std::vector<TargetSet> m_target_sets;
        const SolverConfig m_config;
        const EnergyCalculator m_energy_calculator;

        solution_t greedy_random();

        double get_path_cost(const std::vector<Target> &path);

        /*!
         * Gwt paths for all the drones from the specified problem solution
         * @param solution  Problem solution as path consisting of Targets that need to be visited by each drone
         * @return Vector of paths for each drone (size if config.n_uavs)
         */
        std::vector<std::vector<point_t>> get_drones_paths(const solution_t &solution);

        /*!
         * Get path as a sequence of points to be visited from the sequence of targets to be visited
         * @param targets Sequence of targets to be visited by drone
         * @return Sequence of points to be visited by drone
         */
        std::vector<point_t> get_path_from_targets(const std::vector<Target> &targets);

    public:
        MstspSolver(SolverConfig config,
                    const std::vector<MapPolygon> &decomposed_polygons,
                    const EnergyCalculator &energy_calculator);

        std::vector<std::vector<point_t>> solve();
    };
}


#endif //THESIS_TRAJECTORY_GENERATOR_MSTSPSOLVER_H
