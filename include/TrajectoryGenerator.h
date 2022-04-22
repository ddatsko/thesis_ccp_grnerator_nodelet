#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* other important includes */
#include <nav_msgs/Odometry.h>

/* user includes */
#include <mrs_lib/subscribe_handler.h>
#include <mrs_msgs/Path.h>
#include <std_msgs/String.h>
#include <vector>
#include "EnergyCalculator.h"
#include <thesis_trajectory_generator/GeneratePaths.h>
#include "utils.hpp"

namespace trajectory_generatiion {

/* class TrajectoryGenerator //{ */
    class TrajectoryGenerator : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_is_initialized = false;

        /* ros parameters */
        std::string m_uav_name;
        std::string m_kml_file_path;
        bool m_simulation;
        double m_simulation_start_lat;
        double m_simulation_start_long;
        double m_drones_altitude;
        energy_calculator_config_t m_energy_config;

        /* other parameters */
        int sequence_counter = 0;

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
      

        ros::ServiceClient m_trajectory_generator_service_client;

        ros::ServiceServer m_generate_paths_service_server;
        bool callback_generate_paths(thesis_trajectory_generator::GeneratePaths::Request &req, thesis_trajectory_generator::GeneratePaths::Response &res);


        /*!
         * Generate path for one drone flying in a simulation
         * @param points_to_visit Points that need to be visited in the appropriate order
         * @param max_distance_between_points Max distance between two consecutive points. If initial distance between
         * consecutive points is larger, new ones will be added
         * @return Path that can be sent to the follower or trajectory generator to follow it
         */
        mrs_msgs::Path _generate_path_for_simulation_one_drone(const std::vector<std::pair<double, double>> &points_to_visit,
                                                               point_t gps_transform_origin,
                                                               double distance_for_turning = std::numeric_limits<double>::max(),
                                                               int max_number_of_extra_points = 0,
                                                               double optimal_speed=1);
    };
//}

}  // namespace trajectory_generatiion
