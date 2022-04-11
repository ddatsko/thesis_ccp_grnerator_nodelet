#include <TrajectoryGenerator.h>

// TODO: make a separate executable from all this staff that does not use any ROS

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

        pl.loadParam("UAV_NAME", m_uav_name);
        pl.loadParam("KML_FILE_PATH", m_kml_file_path);
        pl.loadParam("SIMULATION", m_simulation);
        pl.loadParam("SIMULATION_START_LAT", m_simulation_start_lat);
        pl.loadParam("SIMULATION_START_LONG", m_simulation_start_long);
        pl.loadParam("ALTITUDE", m_drones_altitude);
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

        m_trajectory_generator_service_client = nh.serviceClient<mrs_msgs::PathSrv>("/" + m_uav_name + "/path_to_follow");

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("TrajectoryGenerator", m_uav_name);

        MapPolygon polygon;
        try {
            polygon.load_polygon_from_file(m_kml_file_path);
        } catch (kml_file_parse_error &e) {
            ROS_ERROR_STREAM("[TrajectoryGenerator]: " << e.what());
            ros::shutdown();
            return;
        }
//        std::cout << "Points: " << std::endl;
//        for (auto &p: polygon.fly_zone_polygon_points) {
//            std::cout << p.first << " " << p.second << std::endl;
//        }

        ShortestPathCalculator shortest_path_calculator{polygon};

        std::vector<MapPolygon> polygons_decomposed;
        try {
            polygons_decomposed = trapezoidal_decomposition(polygon, BOUSTROPHEDON_WITH_CONVEX_POLYGONS);
        } catch (const polygon_decomposition_error &e) {
            std::cout << e.what() << std::endl;
        }

        {
            std::ofstream of{"/home/mrs/polygons/main.csv"};
            for (auto &p: polygon.fly_zone_polygon_points) {
                of << std::setprecision(10) << p.first << ", " << std::setprecision(10) << p.second << std::endl;
            }
            of.close();
        }

        std::vector<MapPolygon> polygons_divided;
        for (auto &p: polygons_decomposed) {
            auto divided = p.split_into_pieces(700000);
            polygons_divided.insert(polygons_divided.end(), divided.begin(), divided.end());
        }
        polygons_decomposed = polygons_divided;

        int counter = 0;
        for (auto &pol: polygons_decomposed) {
            std::stringstream filename;
            filename << "/home/mrs/polygons/" << counter++ << ".csv";
            std::ofstream of{filename.str()};
            for (auto &p: pol.fly_zone_polygon_points) {
                of << std::setprecision(10) << p.first << ", " << std::setprecision(10) << p.second << std::endl;
            }
            of.close();
        }
        std::cout << "Decomposed..." << std::endl;

        EnergyCalculator energy_calculator(m_energy_config);
        point_t starting_point = gps_coordinates_to_meters({m_simulation_start_long, m_simulation_start_lat});
        mstsp_solver::SolverConfig solver_config {{0, M_PI_2,}, 10, starting_point, 3};
        mstsp_solver::MstspSolver solver(solver_config, polygons_decomposed, energy_calculator, shortest_path_calculator);

        auto uav_paths = solver.solve();


        for (size_t i = 0; i < uav_paths.size(); ++i) {
            std::stringstream filename_ss;
            filename_ss << "/home/mrs/polygons/uav" << i << ".csv";
            std::ofstream of(filename_ss.str());
            for (const auto &p: uav_paths[i]) {
                auto point_gps = meters_to_gps_coordinates(p);
                of << std::setprecision(10) << point_gps.first << ", " << std::setprecision(10) << point_gps.second << std::endl;
            }
        }

        ROS_INFO("[TrajectoryGenerator]: successfully loaded polygon from KML file");

        // TODO: make the step parameter be loaded by the param loader, but converted to meters,
        // so it is more convenient to convert it to the drone altitude
//         Graph g(polygon, M_PI / 4, 0.00007);




        if (m_simulation) {
//          auto path_points = sweeping(g);
          if (uav_paths[0].empty()) {
              ROS_ERROR("[TrajectoryGenerator]: algorithms was not able to generate the path");
              ros::shutdown();
              return;
          }


 //         estimate_path_energy_consumption(path_points, 0.9, 4, 0.119, battery, 0.0215);

          size_t longest_path_ind = 0;
          size_t longest_path = 0;
          for (size_t i = 0; i < uav_paths.size(); ++i) {
              if (uav_paths[i].size() > longest_path) {
                  longest_path  = uav_paths[i].size();
                  longest_path_ind = i;
              }
          }
          ROS_INFO_STREAM("[TrajectoryGenerator]: Generated path of length " << uav_paths[longest_path_ind].size() << "  sending it to the drone");
          auto path_to_follow = _generate_path_for_simulation_one_drone(uav_paths[longest_path_ind], 11);

//          EnergyCalculator energy_calculator(m_energy_config);

//          std::cout << "Calculated path energy consumption: " << energy_calculator.calculate_path_energy_consumption(path_points) << std::endl;


          mrs_msgs::PathSrv srv;

          srv.request.path = path_to_follow;
          if (m_trajectory_generator_service_client.call(srv)) {
            if (not srv.response.success) {
              ROS_ERROR("[TrajectoryGenerator]: Path could not be generated");
              ros::shutdown();
              return;
            } else {
              ROS_INFO("[TrajectoryGenerator]: Path was generated successfully and drone should start following it");
            }
          } else {
            ROS_ERROR_STREAM ("[TrajectoryGenerator]: Could not call the service for generating path for the drone: " << srv.response.message);
          }
        } else {
          ROS_ERROR("[TrajectoryGenerator]: fuunctionality for real world is not implemented yed");
          ros::shutdown();
          return;
        }

        ROS_INFO_ONCE("[TrajectoryGenerator]: initialized");

        m_is_initialized = true;
    }
//}

mrs_msgs::Path TrajectoryGenerator::_generate_path_for_simulation_one_drone(std::vector<std::pair<double, double>> &points_to_visit,
                                                                            double max_distance_between_points) {
  mrs_msgs::Path path;
  if (not m_simulation || points_to_visit.empty()) {
    // Return an empty message if the algorithm is running not for a simulation
    return path;
  }
  std::cout << "Points to visit: " << points_to_visit.size() << std::endl;
  std::vector<std::pair<double, double>> points_to_visit_dense;
  points_to_visit_dense.push_back(points_to_visit.front());
  for (size_t i = 1; i < points_to_visit.size(); ++i) {
      double distance = distance_between_points(points_to_visit[i - 1], points_to_visit[i]);
      int new_points_in_segment = std::ceil(std::max(0.0, distance / max_distance_between_points - 1));
      double dx = (points_to_visit[i].first - points_to_visit[i - 1].first) / (new_points_in_segment + 1);
      double dy = (points_to_visit[i].second - points_to_visit[i - 1].second) / (new_points_in_segment + 1);

//      points_to_visit_dense.push_back(points_to_visit[i - 1]);
      for (int j = 1; j < new_points_in_segment + 2; ++j) {
          points_to_visit_dense.emplace_back(points_to_visit[i - 1].first + dx * j, points_to_visit[i - 1].second + dy * j);
      }
  }

  // Set the parameters for trajectory generation
  path.header.stamp = ros::Time::now();
  path.header.seq = sequence_counter++;
  path.header.frame_id = "latlon_origin";

  path.fly_now = true;
  path.use_heading = false;
  path.stop_at_waypoints = false;
  path.loop = false;
  path.override_constraints = false;

  // TODO: find out what this parameter means
  path.relax_heading = true;

  std::vector<mrs_msgs::Reference> points;

  for (auto p: points_to_visit_dense) {
    mrs_msgs::ReferenceStamped point_3d;
    point_3d.header.frame_id = "latlon_origin";
    point_3d.reference.heading = 6;

    p = meters_to_gps_coordinates(p);
    std::cout << "GPS: " << p.first << " " << p.second << std::endl;
    point_3d.reference.position.x = p.second;
    point_3d.reference.position.y = p.first;
    point_3d.reference.position.z = m_drones_altitude;

    points.push_back(point_3d.reference);
//    auto transformed_reference = m_transformer.transformSingle("utm_origin", point_3d);
//    if (!transformed_reference.has_value()) {
//        ROS_ERROR("Could not transform point");
//        return path;
//    }
//    std::cout << transformed_reference->reference.position.x << " " << transformed_reference->reference.position.y << std::endl;
//    transformed_reference->reference.heading = 6;
//    points.push_back(transformed_reference->reference);
  }
  path.points = points;
  return path;



}



}  // namespace trajectory_generatiion

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(trajectory_generatiion::TrajectoryGenerator, nodelet::Nodelet)
