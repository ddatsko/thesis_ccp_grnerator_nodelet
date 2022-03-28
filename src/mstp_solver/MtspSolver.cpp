//
// Created by mrs on 28.03.22.
//

#include "mstp_solver/MtspSolver.h"
#include <utility>
#include "mstp_solver/Insertion.h"
#include <cstdlib>

MtspSolver::MtspSolver(SolverConfig config, const std::vector<MapPolygon> &decomposed_polygons,
                       const EnergyCalculator &energy_calculator): m_config(std::move(config)), m_energy_calculator(energy_calculator) {

    for (size_t i = 0; i < decomposed_polygons.size(); ++i) {
        m_target_sets.emplace_back(i, decomposed_polygons[i], m_config.sweeping_step, m_energy_calculator, config.rotation_angles);
    }
}


solution_t MtspSolver::greedy_random() {
    solution_t current_solution(m_config.n_uavs);
    auto target_sets = m_target_sets;

    // initial search in close neighborhood
    // - Find all the possible insertions of each target in each path
    // - Calculate the new paths cost of the insertion
    // - Take 1/4 of best insertions
    // - Randomly choose one of them and insert it
    // - Go to first step
    while (!target_sets.empty()) {
        std::set<Insertion, InsertionComp> possible_insertions;
        for (size_t i = 0; i < target_sets.size(); ++i) {
            for (size_t j = 0; j < m_config.n_uavs; ++j) {
                for (size_t k = 0; k <= current_solution[j].size(); ++k) {
                    // TODO: maybe, choose just one insertion from all of the next loop
                    for (size_t target_id = 0; target_id < target_sets[i].targets.size(); ++target_id) {
                        std::vector<Target> current_route = current_solution[j];
                        current_route.insert(current_route.begin() + static_cast<long>(k), target_sets[i].targets[target_id]);
                        double cost = get_path_cost(current_route);
                        possible_insertions.insert({cost, i, target_id, j, k});
                    }
                }
            }
        }

        size_t size_reduced = possible_insertions.size() / 4;
        // Could generate random numbers better, but let it be. We don't need a perfect uniformity
        size_t random = std::rand() & (size_reduced + 1);
        if (random >= possible_insertions.size()) {
            random = possible_insertions.size() - 1;
        }
        Insertion chosen_insertion = *std::next(possible_insertions.begin(), random);

        current_solution[chosen_insertion.uav_index].emplace(current_solution[chosen_insertion.uav_index].begin() + static_cast<long>(chosen_insertion.insertion_index),
                                                             target_sets[chosen_insertion.target_index].targets[chosen_insertion.target_index]);
        target_sets.erase(target_sets.begin() + chosen_insertion.target_set_index);
    }
    return current_solution;
}


double MtspSolver::get_path_cost(const std::vector<Target> &path) {
    double energy = 0;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        energy += path[i].energy_consumption;
        energy += m_energy_calculator.calculate_straight_line_energy( 0, 0, path[i].end_point, path[i + 1].starting_point);
    }
    energy += path[path.size() - 1].energy_consumption;
    return energy;
}


solution_t MtspSolver::solve() {
    solution_t random_solution = greedy_random();
}
