#include <PathGenerator.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
#include <mrs_msgs/PathSrv.h>
#include <std_msgs/String.h>
#include "MapPolygon.hpp"
#include "EnergyCalculator.h"
#include "algorithms.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>
#include "utils.hpp"
#include "ShortestPathCalculator.hpp"
#include "mstsp_solver/SolverConfig.h"
#include "mstsp_solver/MstspSolver.h"
#include <thesis_path_generator/GeneratePaths.h>

namespace path_generation {

/* onInit() method //{ */
    void PathGenerator::onInit() {

        // | ---------------- set my booleans to false ---------------- |
        // but remember, always set them to their default value in the header file
        // because, when you add new one later, you might forget to come back here

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |<<6
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */
        mrs_lib::ParamLoader pl(nh, "PathGenerator");

        pl.loadParam("battery_model/cell_capacity", m_energy_config.battery_model.cell_capacity);
        pl.loadParam("battery_model/number_of_cells", m_energy_config.battery_model.number_of_cells);
        pl.loadParam("battery_model/d0", m_energy_config.battery_model.d0);
        pl.loadParam("battery_model/d1", m_energy_config.battery_model.d1);
        pl.loadParam("battery_model/d2", m_energy_config.battery_model.d2);
        pl.loadParam("battery_model/d3", m_energy_config.battery_model.d3);

        pl.loadParam("best_speed_model/c0", m_energy_config.best_speed_model.c0);
        pl.loadParam("best_speed_model/c1", m_energy_config.best_speed_model.c1);
        pl.loadParam("best_speed_model/c2", m_energy_config.best_speed_model.c2);
        pl.loadParam("drone_mass", m_energy_config.drone_mass);
        pl.loadParam("drone_area", m_energy_config.drone_area);
        pl.loadParam("average_acceleration", m_energy_config.average_acceleration);
        pl.loadParam("propeller_radius", m_energy_config.propeller_radius);
        pl.loadParam("number_of_propellers", m_energy_config.number_of_propellers);
        pl.loadParam("allowed_path_deviation", m_energy_config.allowed_path_deviation);


        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[PathGenerator]: failed to load non-optional parameters!");
            ros::shutdown();
            return;
        } else {
            ROS_INFO_ONCE("[PathGenerator]: loaded parameters");
        }

        m_generate_paths_service_server = nh.advertiseService("/generate_paths",
                                                              &PathGenerator::callback_generate_paths, this);

        ROS_INFO_ONCE("[PathGenerator]: initialized");

        m_is_initialized = true;
    }

    bool PathGenerator::callback_generate_paths(thesis_path_generator::GeneratePaths::Request &req,
                                                thesis_path_generator::GeneratePaths::Response &res) {

        if (!m_is_initialized) return false;
        res.success = false;
        if (req.fly_zone.points.size() <= 2) {
            res.message = "Fly zone of less than 3 points";
            return true;
        }
        m_drones_altitude = req.drones_altitude;
        m_unique_altitude_step = req.unique_altitude_step;

        point_t gps_transform_origin{req.fly_zone.points.front().x, req.fly_zone.points.front().y};

        // Convert the area from message to custom MapPolygon type
        MapPolygon polygon{req.fly_zone, req.no_fly_zones, gps_transform_origin};

        // Override UAV and battery parameters if it is stated to do so in request
        energy_calculator_config_t energy_config = m_energy_config;
        if (req.override_battery_model) {
            energy_config.battery_model.cell_capacity = req.battery_cell_capacity;
            energy_config.battery_model.number_of_cells = req.battery_number_of_cells;
        }
        if (req.override_drone_parameters) {
            ROS_INFO("[PathGenerator] Overriding UAV parameters");
            energy_config.drone_area = req.drone_area;
            energy_config.average_acceleration = req.average_acceleration;
            energy_config.drone_mass = req.drone_mass;
            energy_config.number_of_propellers = req.number_of_propellers;
            energy_config.propeller_radius = req.propeller_radius;
        }

        // Create the energy calculator from user-defines parameters
        EnergyCalculator energy_calculator{energy_config};
        ROS_INFO_STREAM("[PathGenerator]: Optimal speed: " << energy_calculator.get_optimal_speed());

        // Decompose the polygon
        ShortestPathCalculator shortest_path_calculator(polygon);

        if (req.decomposition_method >= static_cast<uint8_t>(DECOMPOSITION_TYPES_NUMBER)) {
            ROS_ERROR_STREAM("[PathGenerator]: Wrong decomposition method chosen");
            res.message = "Wrong decomposition method";
            return true;
        }


        mstsp_solver::final_solution_t best_solution;
        try {
            best_solution = solve_for_uavs(req.number_of_drones, req, polygon, energy_calculator, shortest_path_calculator, gps_transform_origin);
        } catch (const polygon_decomposition_error &e) {
            ROS_ERROR("[PathGenerator]: Error while decomposing the polygon");
            res.success = false;
            res.message = "Error while decomposing the polygon";
            return true;
        }
        auto best_paths = best_solution.paths;

        // Modify the finishing coordinate to prevent drones from collision at the end.
        // TODO: move it to planner directly to prevent any enterings of no-fly zones while travelling to the end point
        for (size_t i = 0; i < best_paths.size(); ++i) {
            best_paths[i].back().x += static_cast<double>(i) * req.end_point_x_difference;
        }

        // Make up a response
        res.success = true;
        res.paths_gps.resize(best_paths.size());
        res.energy_consumptions.resize(best_paths.size());
        for (size_t i = 0; i < best_paths.size(); ++i) {
            res.paths_gps[i].header.frame_id = "latlon_origin";
            res.energy_consumptions[i] = energy_calculator.calculate_path_energy_consumption(
                    remove_path_heading(best_paths[i]));

            // Convert each path to Path message in "latlon_origin" frame
            auto generated_path = _generate_path_for_simulation_one_drone(best_paths[i], gps_transform_origin,
                                                                          energy_calculator.get_optimal_speed(),
                                                                          energy_config.average_acceleration);
            res.paths_gps[i] = generated_path;
        }
        return true;
    }


    mrs_msgs::Path PathGenerator::_generate_path_for_simulation_one_drone(
            const std::vector<point_heading_t<double>> &points_to_visit,
            point_t gps_transform_origin,
            double optimal_speed,
            double horizontal_acceleration) {
        mrs_msgs::Path path;

        // Set the parameters for trajectory generation
        path.header.stamp = ros::Time::now();
        path.header.seq = sequence_counter++;
        path.header.frame_id = "latlon_origin";

        path.fly_now = true;
        path.use_heading = true;
        path.stop_at_waypoints = false;
        path.loop = false;
        path.override_constraints = true;

        path.override_max_velocity_horizontal = optimal_speed;
        path.override_max_acceleration_horizontal = horizontal_acceleration;
        path.override_max_jerk_horizontal = 200;
        path.override_max_jerk_vertical = 200;
        path.override_max_acceleration_vertical = 1;
        path.override_max_velocity_vertical = optimal_speed;

        // TODO: find out what this parameter means
        path.relax_heading = false;

        std::vector<mrs_msgs::Reference> points;

        // Convert each point back to "latlon_origin" considering the origin point
        for (auto p: points_to_visit) {
            mrs_msgs::ReferenceStamped point_3d;
            point_3d.header.frame_id = "latlon_origin";

            auto gps_coordinates = meters_to_gps_coordinates({p.x, p.y}, gps_transform_origin);
            point_3d.reference.position.x = gps_coordinates.first;
            point_3d.reference.position.y = gps_coordinates.second;
            point_3d.reference.position.z = p.z;
            ROS_DEBUG_STREAM("Setting heading to " << -p.heading + M_PI_2);

            // Modify the heading to match the sweeping direction after conversion back to "gps_origin"  of a UAV
            point_3d.reference.heading = -p.heading + M_PI_2;

            points.push_back(point_3d.reference);
        }
        path.points = points;
        return path;
    }


    [[maybe_unused]] mstsp_solver::final_solution_t
    PathGenerator::solve_for_uavs(int n_uavs, const thesis_path_generator::GeneratePaths::Request &req,
                                  MapPolygon polygon,
                                  const EnergyCalculator &energy_calculator,
                                  const ShortestPathCalculator &shortest_path_calculator,
                                  std::pair<double, double> gps_transform_origin) {
        auto init_polygon = polygon;
        // TODO: make a parameter taken from message here as it directly influences the computation time
        auto best_initial_rotations = n_best_init_decomp_angles(polygon, 4,
                                                                static_cast<decomposition_type_t>(req.decomposition_method));

        // Run algorithm for each rotation and save the best result
        double best_solution_cost = std::numeric_limits<double>::max();
        mstsp_solver::final_solution_t best_solution;
        for (const auto &rotation: best_initial_rotations) {
            // Decompose polygon using initial rotation
            polygon = init_polygon.rotated(rotation);
            std::vector<MapPolygon> polygons_decomposed;
            polygons_decomposed = trapezoidal_decomposition(polygon,
                                                            static_cast<decomposition_type_t>(req.decomposition_method));

            ROS_INFO_STREAM("[PathGenerator]: Polygon decomposed. Decomposed polygons: ");
            for (const auto &p: polygons_decomposed) {
                ROS_INFO_STREAM("[PathGenerator] Decomposed sub polygon area: " << p.area());
            }

            // Divide large polygons into smaller ones to meet the constraint on the lowest number of sub polygons
            ROS_INFO_STREAM("[PathGenerator]: Dividing large polygons into smaller ones");
            auto polygons_divided = split_into_number(polygons_decomposed,
                                                      static_cast<size_t>(n_uavs) * req.min_sub_polygons_per_uav);
            for (auto &p: polygons_divided) {
                p = p.rotated(-rotation);
            }
            polygons_decomposed = polygons_divided;
            ROS_INFO_STREAM("[PathGenerator]: Divided large polygons into smaller ones");

            // Create the configuration for MSTSP solver
            auto starting_point = gps_coordinates_to_meters({req.start_lat, req.start_lon}, gps_transform_origin);
            mstsp_solver::SolverConfig solver_config{req.rotations_per_cell, req.sweeping_step, starting_point,
                                                     static_cast<size_t>(n_uavs), m_drones_altitude,
                                                     m_unique_altitude_step,
                                                     req.no_improvement_cycles_before_stop};
            solver_config.wall_distance = req.wall_distance;
            mstsp_solver::MstspSolver solver(solver_config, polygons_decomposed, energy_calculator,
                                             shortest_path_calculator);


            auto solver_res = solver.solve();

            // Change the best solution if the current one is better
            if (solver_res.max_path_energy < best_solution_cost) {
                best_solution_cost = solver_res.max_path_energy;
                best_solution = solver_res;
                ROS_INFO_STREAM("[PathGenerator]: best solution rotation: " << rotation / M_PI * 180 << std::endl);
            }
        }

        return best_solution;
    }

}  // namespace trajectory_generatiion

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(path_generation::PathGenerator, nodelet::Nodelet)
