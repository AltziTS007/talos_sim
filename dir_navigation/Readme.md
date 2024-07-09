# Autonomous Navigation

## Table of Contents

- [Introduction](#introduction)
- [Sensors](#sensors)
  - [RGB-D Camera: Intel RealSense D435](#rgbd)
  - [2D LiDARs: Sick TIM561](#2d_lidars)
  - [Odometry: Dynamixel Motors](#odometry)
  - [IMU: LSM9DS1](#imu)
- [Localization and Mapping](#localization)
  - [Localization](#localization)
  - [Mapping](#mapping)
- [Path Planning](#path)
  - [Overview of Path Planning](#overview)
  - [Global Path Planning](#global)
  - [Local Path Planning](#local)
  - [Path Execution and Monitorin](#path_execution)
- [How to use](#how_to)
- [Contributing](#contributing)

## Introduction
In the rapidly evolving field of robotics, autonomous navigation stands as a pivotal capability, enabling robots to traverse and interact with their environment without human intervention. This functionality is essential for applications ranging from warehouse automation and delivery services to exploration in hazardous environments and assistance in healthcare settings. At the heart of many autonomous navigation systems lies the Robot Operating System (ROS), an open-source middleware suite that provides a robust framework for developing complex robotic applications.

## Sensors
Effective autonomous navigation hinges on the seamless integration of a variety of sensors, each providing critical data about the robot's environment and its own state. In our project, we utilize an array of advanced sensors to enhance the robot's perception and navigation capabilities, including an RGB-D camera, 2D LiDARs, odometry from Dynamixel motors, and an IMU. This section provides an overview of each sensor, detailing their roles and contributions to the autonomous navigation system.

### RGB-D Camera: Intel RealSense D435

The Intel RealSense D435 is a versatile RGB-D camera that combines depth sensing with color imaging. It features a wide field of view and high-resolution depth perception, making it ideal for various applications in autonomous navigation. The D435 provides accurate 3D data, enabling the robot to understand the spatial layout of its environment and detect obstacles at different distances. This depth information is crucial for tasks such as mapping, object recognition, and obstacle avoidance.

### 2D LiDARs: Sick TIM561

The Sick TIM561 2D LiDAR sensors are employed for their precise range measurements and reliability in various lighting conditions. These LiDARs provide a 270-degree field of view and deliver accurate distance readings within their detection range. By generating a detailed 2D scan of the environment, the TIM561 LiDARs contribute to real-time mapping and localization, helping the robot to identify and navigate around obstacles with high precision.

### Odometry: Dynamixel Motors

Odometry data from Dynamixel motors is fundamental for tracking the robot's movement and estimating its position over time. The Dynamixel motors, equipped with encoders, provide feedback on the wheel rotations, allowing the calculation of linear and angular displacement. This odometric information is integrated with other sensor data to enhance the accuracy of the robot's localization, ensuring it can follow planned paths and make precise movements.

### IMU: LSM9DS1

The LSM9DS1 Inertial Measurement Unit (IMU) plays a critical role in sensing the robot's orientation and motion dynamics. It includes a 3-axis accelerometer, a 3-axis gyroscope, and a 3-axis magnetometer, providing comprehensive data on linear acceleration, angular velocity, and magnetic field strength. This IMU data is essential for maintaining stable navigation, as it helps to correct drifts in odometry and improves the overall robustness of the localization process.

## Localization and Mapping

Localization and mapping are cornerstone technologies in autonomous navigation, enabling robots to understand and navigate their environment with precision. In our project, we employ a robust combination of sensors and algorithms within the ROS framework to achieve reliable localization and mapping, ensuring the robot can operate autonomously and efficiently. This section delves into the methodologies and technologies used for localization and mapping in our system.

### Localization

Localization is the process of determining the robot’s position and orientation within a given map. Accurate localization is essential for autonomous navigation as it allows the robot to understand where it is in relation to its environment and plan its movements accordingly.

### Mapping

Mapping involves creating a representation of the environment that the robot can use for navigation. This map is a 2D grid that highlights the locations of obstacles and free spaces. Our system uses data from the odometry and the Sick TIM561 2D LiDARs to build and maintain accurate maps.

## Path Planning

Path planning is a crucial component of autonomous navigation, enabling robots to determine and follow optimal routes from their current location to a desired destination. Effective path planning ensures that robots can navigate safely and efficiently, avoiding obstacles and adapting to dynamic environments. In our project, we leverage the capabilities of the ROS framework and the `move_base` package to implement robust path planning strategies. This section explores the methodologies and technologies employed for path planning in our system.

### Overview of Path Planning

Path planning involves creating a sequence of movements that a robot must follow to reach its target while avoiding obstacles. This process can be divided into two main stages:

1. **Global Path Planning**: Determines the overall route from the starting point to the goal, considering the static map of the environment.
2. **Local Path Planning**: Adjusts the global path in real-time to navigate around dynamic obstacles and ensure smooth motion.

Both stages are essential for achieving reliable and adaptable navigation in various environments.

### Global Path Planning

Global path planning involves generating an initial path based on a map of the environment. This map can be a 2D occupancy grid or a 3D point cloud created from sensor data. The global planner in the `move_base` package utilizes algorithms to find the optimal path considering the known obstacles and terrain.

- *A Algorithm*: A common choice for global path planning is the A* (A-star) algorithm. A* searches for the shortest path from the start to the goal by exploring the most promising routes first, based on a cost function that combines the distance traveled and an estimate of the remaining distance.
- **Dijkstra's Algorithm**: Another option is Dijkstra's algorithm, which guarantees finding the shortest path by exploring all possible paths. It is particularly useful in dense maps with many obstacles, although it can be slower than A*.
- **ROS Integration**: The global planner in `move_base` typically uses the `navfn` or `global_planner` packages, which implement these algorithms. These planners take the static map and the robot's start and goal positions as inputs, producing a collision-free path that the robot can follow.

### Local Path Planning

Local path planning is responsible for refining the global path to avoid obstacles detected in real-time. This dynamic adjustment is crucial for navigating in environments with moving objects or changes not captured in the static map.

- **Dynamic Window Approach (DWA)**: The DWA is a popular algorithm for local path planning. It considers the robot's kinematics and generates a set of possible trajectories. Each trajectory is evaluated based on criteria such as distance to the goal, obstacle proximity, and smoothness of motion. The best trajectory is then selected for execution.
- **Time Elastic Band (TEB)**: The TEB local planner optimizes the path by considering both time and space. It models the path as an elastic band that can stretch and bend to avoid obstacles, ensuring smooth and feasible trajectories. TEB is particularly effective in dynamic environments where the robot needs to react quickly to moving obstacles.
- **ROS Integration**: The `dwa_local_planner` and `teb_local_planner` packages in ROS provide implementations of these algorithms. These planners take inputs from the robot's sensors, such as LiDAR and RGB-D cameras, to detect obstacles and adjust the path accordingly.

### Path Execution and Monitoring

The `move_base` package in ROS coordinates the execution of the planned paths, ensuring the robot follows the trajectory accurately and adapts to changes in the environment.

- **Path Following**: The `move_base` package sends velocity commands to the robot's actuators, guiding it along the planned path. It continuously monitors the robot's progress and adjusts commands as necessary to stay on track.
- **Obstacle Avoidance**: Real-time sensor data from LiDARs, RGB-D cameras, and IMUs are used to detect and avoid obstacles. If a new obstacle is detected, the local planner adjusts the path to navigate around it safely.
- **Recovery Behaviors**: In cases where the robot encounters a situation it cannot navigate, `move_base` includes recovery behaviors such as rotating in place or backing up to find a new path. These behaviors help the robot recover from unforeseen challenges and continue towards its goal.

## How to use
(under devel)

## Contributing
(under devel)
