//
// Created by mrs on 28.03.22.
//

#ifndef THESIS_TRAJECTORY_GENERATOR_TARGETSET_H
#define THESIS_TRAJECTORY_GENERATOR_TARGETSET_H

#include "utils.hpp"
#include <vector>
#include "Target.h"
#include <cmath>
#include "MapPolygon.hpp"
#include "EnergyCalculator.h"

namespace mstsp_solver {

    struct TargetSet {
        size_t index;
        MapPolygon polygon;
        std::vector<Target> targets;
        EnergyCalculator energy_calculator;
        double sweeping_step;

        TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step,
                  const EnergyCalculator &energy_calculator) :
                TargetSet(index, polygon, sweeping_step, energy_calculator, std::vector<double>{0, M_PI}) {};

        TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step,
                  const EnergyCalculator &energy_calculator, const std::vector<double> &rotation_angles);

        TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step,
                  const EnergyCalculator &energy_calculator, size_t number_of_edges_rotations);

    private:
        void add_rotation_angles(const std::vector<double>& angles);
    };
}

#endif //THESIS_TRAJECTORY_GENERATOR_TARGETSET_H
