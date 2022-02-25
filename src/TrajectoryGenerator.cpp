#include <TrajectoryGenerator.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
#include <mrs_msgs/PathSrv.h>
#include <std_msgs/String.h>
#include "MapPolygon.hpp"
#include "algorithms.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>
#include "utils.hpp"

long long METERS_IN_LON_DEGREE = 111000;
long long METERS_IN_LAT_DEGREE = 111000;

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

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[TrajectoryGenerator]: failed to load non-optional parameters!");
            ros::shutdown();
            return;
        } else {
            ROS_INFO_ONCE("[TrajectoryGenerator]: loaded parameters");
        }
        // | ---------------- some data post-processing --------------- |

        m_trajectory_generator_service_client = nh.serviceClient<mrs_msgs::PathSrv>("/" + m_uav_name + "/trajectory_generation/path");

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

        ROS_INFO("[TrajectoryGenerator]: successfully loaded polygon from KML file");

        // TODO: make the step parameter be loaded by the param loader, but converted to meters,
        // so it is more convenient to convert it to the drone altitude
        Graph g = polygon.points_inside_polygon(0.000708);

        if (m_simulation) {
          std::vector<std::pair<double, double>> path_points = wavefront(m_simulation_start_lat, m_simulation_start_long, g);
          if (path_points.empty()) {
              ROS_ERROR("[TrajectoryGenerator]: algorithms was not able to generate the path");
              ros::shutdown();
              return;
          }
          
          
 //         estimate_path_energy_consumption(path_points, 0.9, 4, 0.119, battery, 0.0215); 

          ROS_INFO_STREAM("[TrajectoryGenerator]: Generated path of length " << path_points.size() << "  sending it to the drone"); 
          auto path_to_follow = _generate_path_for_simulation_one_drone(path_points);

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

mrs_msgs::Path TrajectoryGenerator::_generate_path_for_simulation_one_drone(std::vector<std::pair<double, double>> &points_to_visit) {
  mrs_msgs::Path path;
  if (not m_simulation) {
    // Return an empty message if the algorithm is running not for a simulation
    return path;
  }
  
  // Set the parameters for trajectory generation
  path.header.stamp = ros::Time::now();
  path.header.seq = sequence_counter++;
  path.header.frame_id = "gps_origin";

  path.fly_now = true;
  path.use_heading = false;
  path.stop_at_waypoints = false;
  path.loop = false;
  path.override_constraints = false;

  // TODO: find out what this parameter means
  path.relax_heading = true;

  std::vector<mrs_msgs::Reference> points;

  for (auto &p: points_to_visit) {
    mrs_msgs::Reference point_3d;
    point_3d.heading = 1;
    point_3d.position.x = (p.first * METERS_IN_LAT_DEGREE) - (m_simulation_start_lat * METERS_IN_LAT_DEGREE);
    point_3d.position.y = (p.second * METERS_IN_LON_DEGREE) - (m_simulation_start_long * METERS_IN_LON_DEGREE);
    point_3d.position.z = m_drones_altitude;
    points.push_back(point_3d);
  }
  path.points = points;
  return path;

}
  


}  // namespace trajectory_generatiion

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(trajectory_generatiion::TrajectoryGenerator, nodelet::Nodelet)
