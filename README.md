# udacity-robotics-project5

The Home Service Robot project consists of a mobile robot autonomously navigating within a mapped environment. The robot first goes to the pickup location and then to the dropoff location. A green square represents the payload that is picked up and dropped off.

The gazebo world was created using the building editor.
The map was created using pgm_map_creator on the gazebo world.
A Turtlebot is used as the mobile robot, which has a horizontal 2D laser scanner and a 3D camera.
Localisation is achieved with the amcl node, which uses Adaptive Monte Carlo Localization to track the robot pose given an initial pose, odometry measurements, laser scans, and the map.
Navigation is achieved with the move_base node, which uses Dijkstra's Shortest Path First algorithm on a global scale to get to the target as well as at a local scale for obstacle avoidance.
