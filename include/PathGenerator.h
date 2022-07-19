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
#include <thesis_path_generator/GeneratePaths.h>
#include "utils.hpp"
#include "mstsp_solver/MstspSolver.h"

namespace path_generation {

/* class TrajectoryGenerator //{ */
    class PathGenerator : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_is_initialized = false;

        /* ros parameters */
        double m_drones_altitude;
        double m_unique_altitude_step;
        energy_calculator_config_t m_energy_config;

        /* other parameters */
        int sequence_counter = 0;

        // | --------------------- MRS transformer -------------------- |

        // | ---------------------- msg callbacks --------------------- |

        ros::ServiceServer m_generate_paths_service_server;

        bool callback_generate_paths(thesis_path_generator::GeneratePaths::Request &req, thesis_path_generator::GeneratePaths::Response &res);


        /*!
         * Generate path for one drone flying in a simulation
         * @param points_to_visit Points that need to be visited in the appropriate order
         * @param max_distance_between_points Max distance between two consecutive points. If initial distance between
         * consecutive points is larger, new ones will be added
         * @return Path that can be sent to the follower or trajectory generator to follow it
         */
        mrs_msgs::Path _generate_path_for_simulation_one_drone(const std::vector<point_heading_t<double>> &points_to_visit,
                                                               point_t gps_transform_origin,
                                                               double distance_for_turning = std::numeric_limits<double>::max(),
                                                               int max_number_of_extra_points = 0,
                                                               double optimal_speed=1,
                                                               double horizontal_acceleration=2.0);
    };
//}

}  // namespace trajectory_generatiion
