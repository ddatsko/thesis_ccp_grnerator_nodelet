#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma ide diagnostic ignored "cert-msc50-cpp"
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


    solution_t MstspSolver::greedy_random() const {
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
            std::cout << "Possible insertions number" << possible_insertions.size() << std::endl;
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
            target_sets.erase(target_sets.begin() + static_cast<long>(chosen_insertion.target_set_index));
        }
        return current_solution;
    }


    double MstspSolver::get_path_cost(const std::vector<Target> &path) const {
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



    std::vector<std::vector<point_t>> MstspSolver::get_drones_paths(const solution_t &solution) const {
        std::vector<std::vector<point_t>> res;
        for (const auto &drone_path: solution) {
            res.push_back(get_path_from_targets(drone_path));
        }
        return res;
    }

    std::vector<point_t> MstspSolver::get_path_from_targets(const std::vector<Target> &targets) const {
        std::vector<point_t> res;
        res.push_back(m_config.starting_point);
        for (const auto &target: targets) {
            auto path = sweeping(m_target_sets[target.target_set_index].polygon, target.rotation_angle,
                                 m_config.sweeping_step, target.first_line_up);
            res.insert(res.end(), path.begin(), path.end());
        }

        res.push_back(m_config.starting_point);
        return res;
    }


    std::vector<std::vector<point_t>> MstspSolver::solve() const {
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

    double MstspSolver::get_solution_cost(const solution_t &solution) const {
        //TODO: tune this function. We should somehow take into account both average and max energy spent by a drone
        // For now, calculate only the sum of energies spent
        double cost = 0;
        for (const auto &uav_path: solution) {
            cost += get_path_cost(uav_path);
        }
        return cost;
    }

    // Random shift intra-inter route
    void MstspSolver::get_g1_solution(solution_t &solution) const {
        for (size_t i = 0; i <= solution.size(); ++i) {
            // If each UAV visits only 1 or 0 polygons, there is no need (and it will lead to some errors) to continue
            if (i == solution.size()) {
                return;
            }
        }
        size_t routes = solution.size();
        size_t index_a1, index_a2;
        do {
            index_a1 = std::rand() % routes;
        } while (solution[index_a1].size() < 2);

        size_t index_c2, index_c1 = std::rand() % solution[index_a1].size();

        Target target_to_move = solution[index_a1][index_c1];
        solution[index_a1].erase(solution[index_a1].begin() + static_cast<long>(index_c1));

        if (std::rand() % 2 == 0) { // Shift intra route
            index_a2 = index_a1;
            do {
                index_c2 = std::rand() % (solution[index_a1].size() + 1);
            } while (index_c1 == index_c2);
            solution[index_a2].emplace(solution[index_a2].begin() + static_cast<long>(index_c2), target_to_move);
        } else {
            do {
                index_a2 = std::rand() % routes;
            } while (index_a2 == index_a1);
            index_c2 = std::rand() % (solution[index_a2].size() + 1);
            solution[index_a2].emplace(solution[index_a2].begin() + static_cast<long>(index_c2), target_to_move);
        }
        // Try to rotate the moved target and find the best rotation
        double min_route_cost = get_path_cost(solution[index_a2]);
        Target best_target = target_to_move;
        for (const auto &rotated_target: m_target_sets[target_to_move.target_set_index].targets) {
            solution[index_a2][index_c2] = rotated_target;
            double route_cost = get_path_cost(solution[index_a2]);
            if (route_cost < min_route_cost) {
                min_route_cost = route_cost;
                best_target = rotated_target;
            }
        }
        solution[index_a2][index_c2] = best_target;
    }

    // best shift intra-inter route based on exhaustive search
    void MstspSolver::get_g2_solution(solution_t &solution) const {
        for (size_t i = 0; i <= solution.size(); ++i) {
            // If each UAV visits only 1 or 0 polygons, there is no need (and it will lead to some errors) to continue
            if (i == solution.size()) {
                return;
            }
        }
        size_t routes = solution.size();
        size_t index_a1;
        do {
            index_a1 = std::rand() % routes;
        } while (solution[index_a1].size() < 2);

        size_t index_c1 = std::rand() % solution[index_a1].size();

        double best_solution_cost = std::numeric_limits<double>::max();
        size_t target_in_target_set_index = 0;

        Target target_to_move = solution[index_a1][index_c1];
        const TargetSet &target_set_to_check = m_target_sets[target_to_move.target_set_index];

        solution[index_a1].erase(solution[index_a1].begin() + static_cast<long>(index_c1));

        for (size_t i = 0; i < solution.size(); ++i) {
            for (size_t j = 0; j <= solution[i].size(); ++j) {
                if (i == index_a1 && j == index_c1) {
                    continue;
                }
                solution_t intermediate_solution = solution;
                intermediate_solution[i].insert(intermediate_solution[i].begin() + static_cast<long>(j), target_to_move);
                for (size_t k = 0; k < target_set_to_check.targets.size(); ++k) {
                    intermediate_solution[i][j] = target_set_to_check.targets[k];
                    // TODO: in Franta's code there is something strange here
                    double path_cost = get_solution_cost(intermediate_solution);
                    if (path_cost < best_solution_cost) {
                        best_solution_cost = path_cost;
                        index_a1 = i;
                        index_c1 = j;
                        target_in_target_set_index = k;
                    }
                }
            }
        }
        solution[index_a1].insert(solution[index_a1].begin() + static_cast<long>(index_c1),
                                  m_target_sets[target_to_move.target_set_index].targets[target_in_target_set_index]);


    }

    // best swap intra-inter route based on exhaustive search
    void MstspSolver::get_g3_solution(solution_t &solution) const {

    }


    void MstspSolver::get_g4_solution(solution_t &solution) const {
        for (size_t i = 0; i <= solution.size(); ++i) {
            // If each UAV visits only 1 or 0 polygons, there is no need (and it will lead to some errors) to continue
            if (i == solution.size()) {
                return;
            }
        }
        size_t routes = solution.size();
        size_t index_a1;
        do {
            index_a1 = std::rand() % routes;
        } while (solution[index_a1].size() < 2);

        size_t index_c1 = std::rand() % solution[index_a1].size();

        double best_path_cost = get_path_cost(solution[index_a1]);
        Target best_target = solution[index_a1][index_c1];
        const TargetSet &target_to_check = m_target_sets[best_target.target_set_index];
        for (const auto &target: target_to_check.targets) {
            solution[index_a1][index_c1] = target;
            double path_cost = get_path_cost(solution[index_a1]);
            if (path_cost < best_path_cost) {
                best_path_cost = path_cost;
                best_target = target;
            }
        }
        solution[index_a1][index_c1] = best_target;
    }

    void MstspSolver::find_best_targets_for_position(solution_t &solution, size_t uav1, size_t path_index_1,
                                          size_t uav2, size_t path_index_2) {
        double best_cost = get_path_cost(solution[uav1]) + get_path_cost(solution[uav2]);
        for (const auto &target1: )

    }


}
#pragma clang diagnostic pop