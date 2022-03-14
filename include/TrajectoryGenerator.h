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

        mrs_msgs::Path _generate_path_for_simulation_one_drone(std::vector<std::pair<double, double>> &points_to_visit);
    };
//}

}  // namespace trajectory_generatiion
