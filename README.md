# udacity-robotics-project5

## Overview
The Home Service Robot project consists of a simulated mobile robot autonomously navigating within a Gazebo environment. The robot first goes to the pickup location and then to the dropoff location. A green square represents the payload that is picked up and dropped off.

## Simulation
The gazebo world was created using the building editor. The map is asymmetric and has geometric features so that the localization is able to successfully determine the position. If the building is symmetric then localization may find different robot poses to be equally likely.

A Turtlebot is used as the mobile robot, which provides odometry and a horizontal 2D laser scanner.

## Localization
Localisation is achieved with the ROS amcl_node. AMCL (Adaptive Monte Carlo Localization) makes use of a particle filter, which continually evaluates and updates a list of randomly placed potential poses. Evaluation of each pose is done by comparing the laser scan projected from that pose with the map, resulting in the likelihood of the pose being correct. The particle filter poses should quickly group around the true robot pose, if AMCL was set up correctly. 

Important parameters for getting AMCL to work properly are odometry noise and odometry turn noise.

The map used for AMCL was created using pgm_map_creator on the gazebo world file that has the building in it.

## Navigation
Navigation is achieved with the move_base node.

which uses Dijkstra's Shortest Path First algorithm on a global scale to get to the target as well as at a local scale for obstacle avoidance.

Important parameters for good results with move_base include the robot size, turn speed and move speed.

The pick_objects node commands the move_node to drive the robot to a pickup location and then a dropoff location. It knows when the pickup location has been reached by waiting for feedback from the move node indicating successful completion.

The add_markers node simulates a virtual object being picked up from the first location and dropped off at the second location. It monitors the reported robot position and updates a virtual marker in rviz to correspond with what is happening.

## Scripts
The home_service.sh script runs the simulation and visualization environments, and the ROS nodes.
