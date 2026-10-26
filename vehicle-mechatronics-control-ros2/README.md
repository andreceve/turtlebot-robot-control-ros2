# Vehicle Mechatronics: Control — ROS 2 & TurtleBot3

Exercises and final project for the course **Vehicle Mechatronics: Control (MEC-E5012)** at
**Aalto University**, Master's programme in Autonomous Systems and Intelligent Robots
(August – October 2026).

All work is done in **ROS 2 Humble** with a simulated **TurtleBot3 Burger** in **Gazebo**,
moving step by step from basic publishers and subscribers to a complete navigation stack:
LiDAR perception, feedback control, A\* path planning, ICP scan matching with occupancy-grid
mapping, and finally autonomous navigation through a maze with blocked passages.

---

## Contents

The assignments are listed in the order they were done in the course.

| # | Folder | Topic | Main techniques |
|---|---|---|---|
| 1 | [`ROS_turtle_development_in_Gazebo`](ROS_turtle_development_in_Gazebo) | First ROS 2 nodes on TurtleBot3 | SLAM map, Nav2 navigation, publishers/subscribers, odometry, ROS 2 services |
| 2 | [`ROS_turtlebot_Perception_Control_and_Path_Planning`](ROS_turtlebot_Perception_Control_and_Path_Planning) | LiDAR-based driving and path planning | LiDAR sector analysis, PD steering control, A\* search |
| 3 | [`ROS_turtlebot_LIDAR_mapping`](ROS_turtlebot_LIDAR_mapping) | Mapping with a 2D LiDAR | Point-to-point ICP (SVD), probabilistic occupancy grid (log-odds) |
| 4 | [`project_work_Cevenini`](project_work_Cevenini) | **Final project:** maze navigation with obstacles | A\* on a grid graph, pure pursuit + PID, LiDAR passability check, backtracking and replanning |

Each folder has its own README with the task description, the approach and the results.

## Highlights

- **Final project** — the robot plans a route through a 5×5 grid maze with A\*, follows it with a
  pure-pursuit controller, checks each passage with the LiDAR, and when a passage is too narrow
  it backs up to the previous node, removes that edge from the graph and replans.
  A screen recording of the full run is included.
- **LiDAR mapping** — my own implementation of point-to-point ICP (centroids, SVD, reflection
  check, homogeneous transform), combined with a quality gate and log-odds occupancy mapping
  to build a map of an unknown room from LiDAR scans only.
- **Reactive driving** — a PD controller that keeps the robot centred on a zig-zag track using
  only LiDAR distances to the left and right walls.

## Tools

ROS 2 Humble · Python (rclpy, NumPy, SciPy, OpenCV, pandas) · Gazebo · RViz · Nav2 ·
TurtleBot3 · Jupyter / pygame

## How to run (general)

Each ROS 2 package goes into the `src` folder of a colcon workspace:

```bash
cd ~/ros2_ws/src
cp -r <path-to-package> .
cd ~/ros2_ws && colcon build --packages-select <package_name>
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

The Gazebo worlds use track models (`exercise4_track`, `project_track`, …) that were provided
with the course and are not included here. The launch files also rely on the standard
`turtlebot3_gazebo` and `gazebo_ros` packages.

## Note on authorship

The course provided the package templates, launch files, worlds and some helper code
(marked in the files with `TODO: YOUR CODE HERE`). The code written inside those sections, the
controllers, the parameter tuning and the reports are my own work; the final project was done
in a team of two students.

## Author

**Andrea Cevenini** — MSc student, Autonomous Systems and Intelligent Robots (EIT Digital double
degree, University of Bologna – Aalto University)
