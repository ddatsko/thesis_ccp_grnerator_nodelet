#include <EnergyCalculator.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "utils.hpp"

namespace {
    const double AIR_DENSITY = 1.225; // [kg/m^3]
    const double EARTH_GRAVITY = 9.8; //[m/s^2, N/kg]
    const double PROPELLER_EFFICIENCY = 0.54; // Usually from 0.5 to 0.7
    const double RANGE_POWER_CONSUMPTION_COEFF = 1.092; // taken from (17), ratio of power consumption when maximizing the range to power consumption on hover
    const double MOTOR_EFFICIENCY = 0.75; // Efficiency of the motor (ratio of electric power converted to mechanical)

    double cot(double angle) {
        return 1 / std::tan(angle);
    }

}

double EnergyCalculator::angle_between_points(std::pair<double, double> p0, std::pair<double, double> p1,
                                              std::pair<double, double> p2) {
    double a = std::pow(p1.first - p0.first, 2) + std::pow(p1.second - p0.second, 2);
    double b = std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2);
    double c = std::pow(p0.first - p2.first, 2) + std::pow(p0.second - p2.second, 2);

    if (((a + b - c) / std::sqrt(4 * a * b) < -1) || ((a + b - c) / std::sqrt(4 * a * b) > 1)) {
        return 0;
    }

    return std::acos((a + b - c) / std::sqrt(4 * a * b));
}

EnergyCalculator::EnergyCalculator(const energy_calculator_config_t &energy_calculator_config) : config(
        energy_calculator_config) {
    // induced velocity at hover
    double v_i_h = std::sqrt((config.drone_mass * EARTH_GRAVITY) /
                             (2 * AIR_DENSITY * M_PI * config.propeller_radius * config.propeller_radius *
                              config.number_of_propellers)); // (4)
//    std::cout << v_i_h << std::endl;
    // Power consumption on hover
//  double P_h = (std::sqrt(config.number_of_propellers) * config.drone_mass * EARTH_GRAVITY * v_i_h) / (PROPELLER_EFFICIENCY); // (5)
    double P_h = std::sqrt(std::pow(config.drone_mass * EARTH_GRAVITY, 3)) / (PROPELLER_EFFICIENCY * config.propeller_radius * std::sqrt(2 * AIR_DENSITY * M_PI * config.number_of_propellers));

    // Power consumption when maximizing the range
    P_r = P_h * RANGE_POWER_CONSUMPTION_COEFF; // (17)

    double P_mot_r = P_r / MOTOR_EFFICIENCY; // (6)

    double P_cell_r = P_mot_r / (config.battery_model.number_of_cells * config.battery_model.cell_capacity); // (14)

    double C_eff_r = config.battery_model.cell_capacity
                     * (config.battery_model.d0
                        + config.battery_model.d1 * P_cell_r
                        + config.battery_model.d2 * std::pow(P_cell_r, 2)
                        + config.battery_model.d3 * std::pow(P_cell_r, 3));

    t_r = (C_eff_r * 3.7 * config.battery_model.number_of_cells * 3600) / (P_mot_r);

    // TODO: find out why we should use cm^2 and not m^2 here. For now, formula gives correct result for cm^2, but why in the paper they use cm^2????
    // Here, convert drone area to cm^2 as the parameter is fitted for this value
    double v_r_inv = config.best_speed_model.c0 + config.best_speed_model.c1 * v_i_h +
                     config.best_speed_model.c2 * (config.drone_area * 10000); // (18)

    v_r = v_i_h / v_r_inv;

    std::cout << "ENERGY CALCULATOR: Optimal speed: " << v_r << "Time of flight: " << t_r << std::endl;
}

turning_properties_t EnergyCalculator::calculate_turning_properties(double angle) const {
    angle = std::abs(angle);

    // If angle is extremely close to pi, to avoid any numerical errors due to numbers close to 0, just assume that no
    // Deceleration or acceleration is needed at all
    if (std::abs(angle * 180 / M_PI - 180) < 1) {
        return {v_r, -config.average_acceleration, v_r, config.average_acceleration, 0};
    }

    // Calculate the new speed, where x coordinate is the first vector direction
    // i.e. v_y == 0; new_vx == v_x if angle = 180 deg; new_vy == v_x if angle = 90 deg
    double phi = M_PI - angle;
    double d_vx = v_r * std::sin(phi);
    double d_vy = std::abs(v_r - std::cos(phi) * v_r);
    double omega_acc = std::asin(d_vy / std::sqrt(d_vx * d_vx + d_vy * d_vy));
    double a_before = -config.average_acceleration * std::sin(omega_acc);
    double a_after = config.average_acceleration * std::cos(omega_acc + (M_PI_2 - phi));


    double v_tx = std::min(d_vx / 2, std::sqrt(2 * config.allowed_path_deviation * config.average_acceleration * std::cos(omega_acc)));
    double v_ty = v_tx / std::tan((M_PI - angle) / 2);

    double energy = config.drone_mass * 0.5 * (std::pow(d_vx, 2) + std::pow(v_r, 2) - std::pow(v_r - d_vy, 2));

    // TODO: it seems, there may be an error here
    double d_vym = std::min(v_ty + (v_tx * cot((M_PI - phi) / 2)), v_r);
//    std::cout << angle << ", " << v_ty << v_r << ", " << std::endl;

    return {v_ty, a_before, v_ty, a_after, energy, d_vym};
}

double EnergyCalculator::calculate_straight_line_energy(double v_in, double a_in, double v_out, double a_out, double s_tot) const {
    // Calculate the time and distance travelled during the acceleration and deceleration phases
    double t_acc = std::abs(v_r - v_in) / a_in;
    double s_acc = v_in * t_acc + 0.5 * a_in * std::pow(t_acc, 2);

    double t_dec = std::abs((v_r - v_out) / a_out);
    double s_dec = v_r * t_dec + 0.5 * a_out * std::pow(t_dec, 2);

    if (s_acc + s_dec <= s_tot) {
        return (t_acc + t_dec + (s_tot - s_acc - s_dec) / v_r) * P_r;
    } else {
        return calculate_short_line_energy(v_in, a_in, v_out, a_out, s_tot);
    }
}

double EnergyCalculator::calculate_straight_line_energy(double v_in, double a_in, double v_out, double a_out, const std::pair<double, double> &p1,
                                                        const std::pair<double, double> &p2) const {
    return calculate_straight_line_energy(v_in, a_in, v_out, a_out, distance_between_points(p1, p2));
}

double EnergyCalculator::calculate_straight_line_energy_between_turns(const turning_properties_t &turn1,
                                                                      const turning_properties_t &turn2,
                                                                      double s_tot) const {

    double t_acc_slow = std::abs(turn1.d_vym - turn1.v_after) / turn1.a_after;
    double s_acc_slow = turn1.v_after * t_acc_slow + 0.5 * turn1.a_after * std::pow(t_acc_slow, 2);

    double t_dec_slow = std::abs((turn2.d_vym - turn2.v_before) / turn2.a_before);
    double s_dec_slow = v_r * t_dec_slow + 0.5 * turn2.a_before * std::pow(t_dec_slow, 2);

    // If the segment is not too short for at least slow acceleration and slow deceleration -- do it
    if (s_acc_slow + s_dec_slow < s_tot) {
        return calculate_straight_line_energy(turn1.d_vym, config.average_acceleration, turn2.d_vym, -config.average_acceleration, s_tot - s_acc_slow - s_dec_slow) +
               P_r * (t_dec_slow + t_acc_slow);
    } else {
        // If the UAV can only start the slow deceleration after the slow acceleration
        return calculate_short_line_energy(turn1.v_after, turn1.a_after, turn2.v_before, turn2.a_before,
                                           s_tot);
    }
}

double EnergyCalculator::calculate_short_line_energy(double v_in, double a_in, double v_out, double a_out, double s) const {
    double a = (1 / (2 * a_in)) + 1 / (2 * a_out);
    double b = - v_r / a_out - v_out / a_out;
    double c = (v_out * v_r) / a_out + v_out * v_out / (2 * a_out) - (v_in * v_in) / (2 * a_in) - s;
    auto solved = solve_quadratic(a, b, c);

    double sol[2] = {solved.first, solved.second};
    for (double si : sol) {
        if (std::isnan(si) || si < v_in || si < v_out) {
            continue;
        }
        double t_acc = (si - v_in) / a_in;
        double s_acc = v_in * t_acc + 0.5 * a_in * t_acc * t_acc;

        double t_dec = std::abs((si - v_out) / a_out);
        double s_dec = v_r * t_dec + 0.5 * a_out * t_dec * t_dec;

        if (s_acc > 0 && s_dec > 0) {
            return (t_acc + t_dec) * P_r;
        }
    }
//    std::cerr << "Warning: Too short segment" << std::endl;
    return 0;
//    return (s / ((v_in + v_out) / 2)) * P_r;
}


double EnergyCalculator::calculate_path_energy_consumption(const std::vector<std::pair<double, double>> &path) const {
    if (path.size() < 2) {
        return 0;
    }
    double total_energy = 0;
    std::vector<turning_properties_t> turns;
    turns.push_back({0, 0, 0, config.average_acceleration, 0});
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        double angle = angle_between_points(path[i - 1], path[i], path[i + 1]);
        turns.push_back(calculate_turning_properties(angle));
    }
    turns.push_back({0, -config.average_acceleration, 0, 0, 0});

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        total_energy += turns[i].energy;
        auto energy = calculate_straight_line_energy_between_turns(turns[i], turns[i + 1], distance_between_points(path[i], path[i + 1]));
        total_energy += energy;
    }
    return total_energy;
}