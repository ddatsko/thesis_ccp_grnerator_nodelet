# ROS node for generating energy-efficient complete coverage pathes for multiple uavs

The node is aimed to work with the [MRS system](https://github.com/ctu-mrs/mrs_uav_system) so it uses some of [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs) for communication.

### Functionality
The node provides two services with customly defined message types: ```/generate_paths``` for path genreation and ```/calculate_energy``` for paths energy calculation.
In both messages, UAV physical parameters can be specified to override default values
