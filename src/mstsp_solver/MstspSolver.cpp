//
// Created by mrs on 28.03.22.
//

#include "mstsp_solver/MstspSolver.h"
#include <utility>
#include "mstsp_solver/Insertion.h"
#include <cstdlib>
#include "algorithms.hpp"
#include <iostream>
#include <algorithm>

namespace mstsp_solver {

    MstspSolver::MstspSolver(SolverConfig config, const std::vector<MapPolygon> &decomposed_polygons,
                             const EnergyCalculator &energy_calculator) : m_config(std::move(config)),
                                                                          m_energy_calculator(energy_calculator) {

        std::cout << "Decomposed polygons: " << decomposed_polygons.size() << std::endl;
        std::cout << "Rotation angles num: " << m_config.rotation_angles.size() << std::endl;
        for (size_t i = 0; i < decomposed_polygons.size(); ++i) {
            m_target_sets.emplace_back(i, decomposed_polygons[i], m_config.sweeping_step, m_energy_calculator,
                                       m_config.rotation_angles);
        }
    }


    solution_t MstspSolver::greedy_random() {
        solution_t current_solution(m_config.n_uavs);
        auto target_sets = m_target_sets;

        // initial search in close neighborhood
        // - Find all the possible insertions of each target in each path
        // - Calculate the new paths cost of the insertion
        // - Take 1/4 of best insertions
        // - Randomly choose one of them and insert it
        // - Go to first step
        while (!target_sets.empty()) {
            std::vector<Insertion> possible_insertions;
            for (size_t i = 0; i < target_sets.size(); ++i) {
                std::cout << "i: " <<  i << std::endl;
                for (size_t j = 0; j < m_config.n_uavs; ++j) {
                    std::cout << "j: " <<  j << std::endl;
                    for (size_t k = 0; k <= current_solution[j].size(); ++k) {
                        std::cout << "k: " <<  k << std::endl;
                        // TODO: maybe, choose just one insertion from all of the next loop
                        for (size_t target_id = 0; target_id < target_sets[i].targets.size(); ++target_id) {
                            std::cout << "target_id: " << target_id << std::endl;
                            std::vector<Target> current_route = current_solution[j];
                            current_route.insert(current_route.begin() + static_cast<long>(k),
                                                 target_sets[i].targets[target_id]);
                            double cost = get_path_cost(current_route);
                            possible_insertions.push_back(Insertion{cost, i, target_id, j, k});
                        }
                    }
                }
            }
            std::sort(possible_insertions.begin(), possible_insertions.end(), [](const Insertion &i1, const Insertion &i2){return i1.solution_cost < i2.solution_cost;});
//            target_sets.erase(target_sets.begin());
            std::cout << "Possisble insertions number" << possible_insertions.size() << std::endl;
            size_t size_reduced = possible_insertions.size() / 4;
            // Could generate random numbers better, but let it be. We don't need a perfect uniformity
            size_t random = std::rand() % (size_reduced + 1);
            if (random >= possible_insertions.size()) {
                random = possible_insertions.size() - 1;
            }
            std::cout << "Chosen insertion index: " << random << std::endl;
            Insertion chosen_insertion = possible_insertions[random];

            std::cout << "Chosen target target set: " << target_sets[chosen_insertion.target_set_index].targets[chosen_insertion.target_index].target_set_index << std::endl;

            current_solution[chosen_insertion.uav_index].emplace(current_solution[chosen_insertion.uav_index].begin() +
                                                                 static_cast<long>(chosen_insertion.insertion_index),
                                                                 target_sets[chosen_insertion.target_set_index].targets[chosen_insertion.target_index]);
            target_sets.erase(target_sets.begin() + chosen_insertion.target_set_index);
        }
        return current_solution;
    }


    double MstspSolver::get_path_cost(const std::vector<Target> &path) {
        if (path.empty()) {
            return 0;
        }
        double energy = 0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            energy += path[i].energy_consumption;
            energy += m_energy_calculator.calculate_straight_line_energy(0, 0, path[i].end_point,
                                                                         path[i + 1].starting_point);
        }
        energy += path[path.size() - 1].energy_consumption;
        energy += m_energy_calculator.calculate_straight_line_energy(0, 0, m_config.starting_point, path[0].starting_point);
        energy += m_energy_calculator.calculate_straight_line_energy(0, 0, path[path.size() - 1].end_point, m_config.starting_point);

        return energy;
    }


    std::vector<std::vector<point_t>> MstspSolver::solve() {
        std::cout << "Solving..." << std::endl;
        solution_t random_solution = greedy_random();
        for (const auto &uav_p: random_solution) {
            std::cout << "UAV PATH: " << std::endl;
            for (const auto &target: uav_p) {
                std::cout << target.target_set_index << std::endl;
            }
        }
        return get_drones_paths(random_solution);
    }

    std::vector<std::vector<point_t>> MstspSolver::get_drones_paths(const solution_t &solution) {
        std::vector<std::vector<point_t>> res;
        for (const auto &drone_path: solution) {
            res.push_back(get_path_from_targets(drone_path));
        }
        return res;
    }

    std::vector<point_t> MstspSolver::get_path_from_targets(const std::vector<Target> &targets) {
        std::vector<point_t> res;
        res.push_back(m_config.starting_point);
        for (const auto &target: targets) {
            auto target_graph = Graph(m_target_sets[target.target_set_index].polygon, target.rotation_angle,
                                      m_config.sweeping_step);
            auto path = sweeping(target_graph, target.first_line_up);
            res.insert(res.end(), path.begin(), path.end());
        }
        res.push_back(m_config.starting_point);
        return res;
    }
}