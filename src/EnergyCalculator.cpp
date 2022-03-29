#include <EnergyCalculator.h>
#include <cmath> 
#include <algorithm>

namespace {
    const double AIR_DENSITY = 1.225; // [kg/m^3]
    const double EARTH_GRAVITY = 9.8; //[m/s^2, N/kg]
    const double PROPELLER_EFFICIENCY = 0.54; // Usually from 0.5 to 0.7
    const double RANGE_POWER_CONSUMPTION_COEFF = 1.092; // taken from (17), ratio of power consumption when maximizing the range to power consumption on hover
    const double MOTOR_EFFICIENCY = 0.75; // Efficiency of the motor (ratio of electric power converted to mechanical)
}

double EnergyCalculator::angle_between_points(std::pair<double, double> p0, std::pair<double, double> p1, std::pair<double, double> p2) const {
  double a = std::pow(p1.first - p0.first, 2) + std::pow(p1.second - p0.second, 2);
  double b = std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2);
  double c = std::pow(p0.first - p2.first, 2) + std::pow(p0.second - p2.second, 2);

  return std::acos((a + b - c) / std::sqrt(4 * a * b));
}

EnergyCalculator::EnergyCalculator(const energy_calculator_config_t &energy_calculator_config): config(energy_calculator_config) {
  // induced velocity at hover
  double v_i_h = std::sqrt((config.drone_mass * EARTH_GRAVITY) / (2 * AIR_DENSITY * M_PI * config.propeller_radius * config.propeller_radius * config.number_of_propellers)); // (4) 
  // Power consumption on hover
  double P_h = (config.drone_mass * EARTH_GRAVITY * v_i_h) / (PROPELLER_EFFICIENCY); // (5)
  
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

  // Here, convert drone area to cm^2 as the parameter is fitted for this value
  double v_r_inv = config.best_speed_model.c0 + config.best_speed_model.c1 * v_i_h + config.best_speed_model.c2 * (config.drone_area * 10000); // (18)
  
  v_r = v_i_h / v_r_inv;
}

double EnergyCalculator::calculate_turning_energy(double angle) const {
  // Calculate the new speed, where x coordinate is the first vector direction
  // i.e. v_y == 0; new_vx == v_x if angle = 180 deg; new_vy == v_x if angle = 90 deg 
  double new_vx = v_r * std::cos(M_PI - angle);
  double new_vy = v_r * std::sin(M_PI - angle);

  double energy_x = (config.drone_mass * std::pow(v_r, 2) / 2) - (config.drone_mass * std::pow(new_vx, 2) / 2);
  double energy_y = (config.drone_mass * std::pow(new_vy, 2) / 2);

  return energy_x + energy_y;
}

double EnergyCalculator::calculate_straight_line_energy(double v_in, double v_out, const std::pair<double, double> &p1, const std::pair<double, double> &p2) const {
  // Calculate the time and distance travelled during the acceleration and deceleration phases
  double t_acc = (v_r - v_in) / config.average_acceleration;
  double s_acc = v_in * t_acc + 0.5 * config.average_acceleration * std::pow(t_acc, 2);
  double t_dec = (v_r - v_out) / config.average_acceleration;
  double s_dec = v_r * t_dec + 0.5 * config.average_acceleration * std::pow(t_dec, 2);

  // Calculate the total distance between two points
  double s_tot = std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));

  // Calculate the total power condumption by finding the time of acceleration + deceleration + constant speed movement
  return (t_acc + t_dec + (s_tot - s_acc - s_dec) / v_r) * P_r;
}

double EnergyCalculator::calculate_path_energy_consumption(const std::vector<std::pair<double, double>> &path) const {
  if (path.size() < 2) {
    return 0;
  }
  double total_energy = 0;
  double v_x = 0, angle;

  // TODO: check if everything is right in this formula
  for (size_t i = 1; i < path.size() - 1; i++) {
    angle = angle_between_points(path[i - 1], path[i], path[i + 1]);
    double new_vx = v_r * std::cos(M_PI - angle);
    total_energy += calculate_straight_line_energy(v_x, std::max(0.0, new_vx), path[i - 1], path[i]);
    total_energy += calculate_turning_energy(angle);
    v_x = new_vx ;
  }
  total_energy += calculate_straight_line_energy(v_x, 0, path[path.size() - 2], path.back());
  return total_energy;
}


