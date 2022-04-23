#include <TrajectoryGenerator.h>

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

#include <thesis_trajectory_generator/GeneratePaths.h>

namespace trajectory_generatiion {

/* onInit() method //{ */
    void TrajectoryGenerator::onInit() {

        // | ---------------- set my booleans to false ---------------- |
        // but remember, always set them to their default value in the header file
        // because, when you add new one later, you might forget to come back here

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |<<6
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */
        mrs_lib::ParamLoader pl(nh, "TrajectoryGenerator");

        pl.loadParam("SIMULATION", m_simulation);
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

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[TrajectoryGenerator]: failed to load non-optional parameters!");
            ros::shutdown();
            return;
        } else {
            ROS_INFO_ONCE("[TrajectoryGenerator]: loaded parameters");
        }
        // | ---------------- some data post-processing --------------- |

        m_trajectory_generator_service_client = nh.serviceClient<mrs_msgs::PathSrv>(
                "/" + m_uav_name + "/path_to_follow");

        m_generate_paths_service_server = nh.advertiseService("/generate_paths",
                                                              &TrajectoryGenerator::callback_generate_paths, this);


        ROS_INFO_ONCE("[TrajectoryGenerator]: initialized");

        m_is_initialized = true;

    }



    bool TrajectoryGenerator::callback_generate_paths(thesis_trajectory_generator::GeneratePaths::Request &req,
                                                      thesis_trajectory_generator::GeneratePaths::Response &res) {

        if (!m_is_initialized) return false;
        res.success = false;
        if (req.fly_zone.points.size() <= 2) {
            res.message = "Fly zone of less than 3 points";
            return true;
        }
        m_drones_altitude = req.drones_altitude;

        point_t gps_transform_origin{req.fly_zone.points.front().x, req.fly_zone.points.front().y};
        // Convert the area from message to custom MapPolygon type
        MapPolygon polygon;
        std::for_each(req.fly_zone.points.begin(), req.fly_zone.points.end(), [&](const auto &p){polygon.fly_zone_polygon_points.push_back(gps_coordinates_to_meters({p.x, p.y}, gps_transform_origin));});
        make_polygon_clockwise(polygon.fly_zone_polygon_points);
        for (auto &no_fly_zone: req.no_fly_zones) {
            polygon.no_fly_zone_polygons.emplace_back();
            std::for_each(no_fly_zone.points.begin(), no_fly_zone.points.end(), [&](const auto &p) {polygon.no_fly_zone_polygons.back().push_back(gps_coordinates_to_meters({p.x, p.y}, gps_transform_origin));});
            make_polygon_clockwise(polygon.no_fly_zone_polygons[polygon.no_fly_zone_polygons.size() - 1]);
        }


        energy_calculator_config_t energy_config = m_energy_config;
        if (req.override_battery_model) {
            energy_config.battery_model.cell_capacity = req.battery_cell_capacity;
            energy_config.battery_model.number_of_cells = req.battery_number_of_cells;
        }

        if (req.override_drone_parameters) {
            ROS_INFO("Overriding UAV parameters");
            energy_config.drone_area = req.drone_area;
            energy_config.average_acceleration = req.average_acceleration;
            energy_config.drone_mass = req.drone_mass;
            energy_config.number_of_propellers = req.number_of_propellers;
            energy_config.propeller_radius = req.propeller_radius;
        }


        // Decompose the polygon
        ShortestPathCalculator shortest_path_calculator(polygon);

        polygon = polygon.rotated(req.decomposition_rotation);
        std::vector<MapPolygon> polygons_decomposed;
        try {
            polygons_decomposed = trapezoidal_decomposition(polygon, BOUSTROPHEDON_WITH_CONVEX_POLYGONS);
        } catch (const polygon_decomposition_error &e) {
            ROS_ERROR("Error while decomposing the polygon");
            res.success = false;
            res.message = "Error while decomposing the polygon";
            return true;
        }

        std::for_each(polygons_decomposed.begin(), polygons_decomposed.end(), [&](MapPolygon &p){p = p.rotated(-req.decomposition_rotation);});
        polygon = polygon.rotated(-req.decomposition_rotation);

        std::vector<MapPolygon> polygons_divided;
        for (auto &pol: polygons_decomposed) {
            std::cout << "Polygon: " << std::endl;
            for (const auto &p: pol.fly_zone_polygon_points) {
                std::cout << p.first << ", " << p.second << std::endl;
            }

            auto splitted = pol.split_into_pieces(req.max_polygon_area != 0 ? req.max_polygon_area : std::numeric_limits<double>::max());
            polygons_divided.insert(polygons_divided.end(), splitted.begin(), splitted.end());
        }
        polygons_decomposed = polygons_divided;

        EnergyCalculator energy_calculator{energy_config};
        auto starting_point = gps_coordinates_to_meters({req.start_lat, req.start_lon}, gps_transform_origin);

        mstsp_solver::SolverConfig solver_config{req.rotations_per_cell, req.sweeping_step, starting_point, req.number_of_drones};
        mstsp_solver::MstspSolver solver(solver_config, polygons_decomposed, energy_calculator,
                                         shortest_path_calculator);

        std::cout << "Optimal speed: " << energy_calculator.get_optimal_speed() << std::endl;

        auto solver_res = solver.solve();
        // Modify the finishing coordinate to prevent drones from collision at the end
        for (size_t i = 0; i < solver_res.size(); ++i) {
            solver_res[i].back().first += static_cast<double>(i) * 3;
        }

        res.success = true;
        res.paths_gps.resize(solver_res.size());
        res.energy_consumptions.resize(solver_res.size());
        for (size_t i = 0; i < solver_res.size(); ++i) {
            res.paths_gps[i].header.frame_id = "latlon_origin";
            res.energy_consumptions[i] = energy_calculator.calculate_path_energy_consumption(solver_res[i]);
            // Do not visit the starting point itself
            solver_res[i].erase(solver_res[i].begin());
            auto generated_path =  _generate_path_for_simulation_one_drone(solver_res[i], gps_transform_origin, req.distance_for_turning, req.max_number_of_extra_points, energy_calculator.get_optimal_speed());
            res.paths_gps[i] = generated_path;
        }


        return true;
    }


    mrs_msgs::Path TrajectoryGenerator::_generate_path_for_simulation_one_drone(
            const std::vector<std::pair<double, double>> &points_to_visit,
            point_t gps_transform_origin,
            double distance_for_turning,
            int max_number_of_extra_points,
            double optimal_speed) {
        mrs_msgs::Path path;
        if (not m_simulation || points_to_visit.empty()) {
            // Return an empty message if the algorithm is running not for a simulation
            return path;
        }

        std::vector<std::pair<double, double>> points_to_visit_dense;
        points_to_visit_dense.push_back(points_to_visit.front());
        for (size_t i = 1; i < points_to_visit.size(); ++i) {
            double distance = distance_between_points(points_to_visit[i - 1], points_to_visit[i]);
            int extra_points_fit = std::floor(distance / distance_for_turning);
            // Add one point before the second point in segment to prevent yawing during the whole segment and constrain it only
            // to the last "distance_for_turning" meters
            for (int j = std::min(extra_points_fit, max_number_of_extra_points); j > 0; --j) {
                points_to_visit_dense.emplace_back(points_to_visit[i].first - j * (points_to_visit[i].first - points_to_visit[i - 1].first) / (distance / distance_for_turning),
                                                   points_to_visit[i].second - j * (points_to_visit[i].second - points_to_visit[i - 1].second) / (distance / distance_for_turning));
            }
            points_to_visit_dense.push_back(points_to_visit[i]);
        }

        // Set the parameters for trajectory generation
        path.header.stamp = ros::Time::now();
        path.header.seq = sequence_counter++;
        path.header.frame_id = "latlon_origin";

        path.fly_now = true;
        path.use_heading = true;
        path.stop_at_waypoints = false;
        path.loop = false;
        path.override_constraints = false;

        path.override_max_velocity_horizontal = optimal_speed;
        path.override_max_acceleration_horizontal = 5;
        path.override_max_jerk_horizontal = 360;
        path.override_max_jerk_vertical = 360;
        path.override_max_acceleration_vertical = 1;
        path.override_max_velocity_vertical = optimal_speed;

        // TODO: find out what this parameter means
        path.relax_heading = false;

        std::vector<mrs_msgs::Reference> points;

        for (auto p: points_to_visit_dense) {
            mrs_msgs::ReferenceStamped point_3d;
            point_3d.header.frame_id = "latlon_origin";

            p = meters_to_gps_coordinates(p, gps_transform_origin);
            point_3d.reference.position.x = p.first;
            point_3d.reference.position.y = p.second;
            point_3d.reference.position.z = m_drones_altitude;

            points.push_back(point_3d.reference);
        }
        path.points = points;
        return path;
    }

}  // namespace trajectory_generatiion

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(trajectory_generatiion::TrajectoryGenerator, nodelet::Nodelet)
