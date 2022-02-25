
#ifndef ENERGY_CALCULATOR_H
#define ENERGY_CALCULATOR_H

#include <vector>

struct battery_model_t {
  double cell_capacity;
  int number_of_cells;

  // Coefficients for the equation https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9691840(21)
  // k = C_eff / C = d_0 + d1 * P_cell + d2 * P_cell^2 + d3 * P_cell^3
  double d0;
  double d1;
  double d2;
  double d3;
};


/*!
 * Constants for determination of the most efficient movement speed.
 * Based on the equation https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9691840 (18)
 */
struct best_speed_model_t {
  double c0;
  double c1;
  double c2;
};


/*! 
 * Structure with pre-calculated constants for the accurate calculation of the energy consumption 
 * */
struct energy_calculator_config_t {
  battery_model_t battery_model;
  best_speed_model_t best_speed_model;
  double drone_mass; // [kg]
  double drone_area; // [m^2]
  double average_acceleration; // [m/s^2]
  double propeller_radius; // [m]
  int number_of_propellers;
};
    

class EnergyCalculator {
private:
  const double AIR_DENSITY = 1.225; // [kg/m^3]
  const double EARTH_GRAVITY = 9.8; //[m/s^2, N/kg]
  const double PROPELLER_EFFICIENCY = 0.54; // Usually from 0.5 to 0.7
  const double RANGE_POWER_CONSUMPTION_COEFF = 1.092; // taken from (17), ratio of power consumption when maximizing the range to power consumption on hover
  const double MOTOR_EFFICIENCY = 0.75; // Efficiency of the motor (ratio of electric power converted to mechanical)

  energy_calculator_config_t config;
  double v_r; // speed at which the drone can cover the longest range [m/s]
  double t_r; // Time during thich the drone can fly with the speed v_r [s]
  double P_r; // Power consumption during movement with the speed v_r

  /*!
   * Calculate the angle between segment (p1, p2) and segment (p2, p3) in radians
   * @param p1: coordinates of a start point
   * @param p2: coordinates of middle point
   * @param p3: coordinates of the third point
   * @return The angle in radians in range (0..PI)
   */
  double angle_between_points(std::pair<double, double> p0, std::pair<double, double> p1, std::pair<double, double> p2);
  
  /*!
   * Calculate the energy spent on turning manuver including the deceleration and acceleration
   *
   * @param angle Turning angle [rad]
   * @return Consumped energy in Joules
   */
  double calculate_turning_energy(double angle);
  
  /*! 
   * Calculate the energy for moving on the straight line. The acceleration and deceleration times are encountered, but the 
   * energy needed to perform the acceleration of deceleration is not encountered (as it should be 
   * enocuntered in the calculate_turning_energy method)
   *
   * @param v_in The speed of the drone while entering the path segment
   * @param v_out The speed of the drone while leaving the path segment
   * @param p1 Start point of the path segment
   * @param p2 End poinr of the path segment
   * @return Energy consumption in Joules
   */ 
  double calculate_straight_line_energy(double v_in, double v_out, const std::pair<double, double> &p1, const std::pair<double, double> &p2);
public:
  EnergyCalculator(const energy_calculator_config_t &energy_calculator_config);
  
  /*!
   * Calculate the total energy spent to follow the path
   * NOTE: the calculation relies on the fact that the closest distance between two path points will allow the drone to
   * fully accelerate to the optimal speed, and decelerate from the optimal speed to 0
   *
   * @param path Path for which the total power consumption will be calculated
   * @return The amount of spent energy to follow the whole path in Joules [J]
   */
  double calculate_path_energy_consumption(const std::vector<std::pair<double, double>> &path);
   
};

#endif
