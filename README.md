# MTE544 Labs - Developing a full robotics stack for Turtlebot4

Implemented autonomous navigation for a Turtlebot 4 using SLAM Toolbox for mapping, particle filter localization, A* path planning with configurable heuristics, and PID trajectory control in Python with ROS2 Humble.

The system acquires environmental maps using SLAM Toolbox with laser scanner data, then enables autonomous point-to-point navigation through particle filter localization, A* path planning, and PID trajectory control. 
* The particle filter estimates robot pose by comparing laser scan measurements against pre-built occupancy grid maps
* The A* planner generates optimal collision-free paths through cost maps with configurable heuristics (Manhattan and Euclidean distance). * The navigation stack converts planned waypoints into velocity commands for differential drive control with PID, enabling the robot to autonomously reach goal poses while avoiding obstacles.

The implementation was validated on physical Turtlebot 4 hardware, demonstrating robust navigation performance across multiple goal positions.
