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

        TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step,
                  const EnergyCalculator &energy_calculator) :
                TargetSet(index, polygon, sweeping_step, energy_calculator, std::vector<double>{0, M_PI}) {};

        TargetSet(size_t index, const MapPolygon &polygon, double sweeping_step,
                  const EnergyCalculator &energy_calculator, const std::vector<double> &rotation_angles);

//    TargetSet(TargetSet &&rhs) = default;
//    TargetSet& operator=(const TargetSet &rhs) = default;
    };
}

#endif //THESIS_TRAJECTORY_GENERATOR_TARGETSET_H
