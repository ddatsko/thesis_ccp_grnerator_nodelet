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

struct metaheuristic_application_error: public std::runtime_error {
    using runtime_error::runtime_error;
};

namespace mstsp_solver {

    using solution_t = std::vector<std::vector<Target>>;

    class MstspSolver {
    private:
        std::vector<TargetSet> m_target_sets;
        const SolverConfig m_config;
        const EnergyCalculator m_energy_calculator;

        solution_t greedy_random() const;

        double get_path_cost(const std::vector<Target> &path) const;

        /*!
         * @param solution Problem solution
         * @return Cost of the solution
         */
        double get_solution_cost(const solution_t &solution) const;

        /*!
         * Gwt paths for all the drones from the specified problem solution
         * @param solution  Problem solution as path consisting of Targets that need to be visited by each drone
         * @return Vector of paths for each drone (size if config.n_uavs)
         */
        std::vector<std::vector<point_t>> get_drones_paths(const solution_t &solution) const;

        /*!
         * Get path as a sequence of points to be visited from the sequence of targets to be visited
         * @param targets Sequence of targets to be visited by drone
         * @return Sequence of points to be visited by drone
         */
        std::vector<point_t> get_path_from_targets(const std::vector<Target> &targets) const;

        void get_g1_solution(solution_t &solution) const;

        void get_g2_solution(solution_t &solution) const;

        void get_g3_solution(solution_t &solution) const;

        void get_g4_solution(solution_t &solution) const;

        void find_best_target_at_positio(solution_t &solution, size_t uav, size_t path_index);

        void find_best_targets_for_position(solution_t &solution, size_t uav1, size_t path_index_1,
                                              size_t uav2, size_t path_index_2);

    public:
        MstspSolver(SolverConfig config,
                    const std::vector<MapPolygon> &decomposed_polygons,
                    const EnergyCalculator &energy_calculator);

        std::vector<std::vector<point_t>> solve() const;
    };
}


#endif //THESIS_TRAJECTORY_GENERATOR_MSTSPSOLVER_H
