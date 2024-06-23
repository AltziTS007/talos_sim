<img title="D.I.R." alt="Alt text" src="/images/dir_horizontal.png">

## Introduction
Welcome to the talos_sim repository! This project provides a simulation environment for the Talos robot, a Mobile Manipulator AMR designed for research and development in robotics, especially for the international RoboCup@Work competition. The repository is tailored for ROS Noetic and includes all the necessary tools to simulate the Talos robot in a virtual environment.

<img title="D.I.R. TALOS" alt="Alt text" src="/images/talos_INVERTED.png">

## Features
- Full simulation of the Talos robot using Gazebo.
- Integration with ROS Noetic.
- Pre-configured robot models and environments (under devel).
- Example launch files to get started quickly.

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
  - [Dependencies](#dependencies)
  - [Setup](#setup)
- [Usage](#usage)
  - [Launching the Simulation](#launching-the-simulation)
  - [Controlling the Robot](#controlling-the-robot)
  - [Available Launch Files](#available-launch-files)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Installation

### Dependencies

Before installing the `talos_sim` package, ensure that you have the following dependencies installed:

- ROS Noetic
- Gazebo
- `catkin` build system
- https://wiki.ros.org/teleop_twist_keyboard
- ```sudo apt-get install ros-noetic-ira-laser-tools```


You can install ROS Noetic by following the official [ROS Noetic installation guide](http://wiki.ros.org/noetic/Installation).

### Setup

1. Clone the repository into your ROS workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone -b noetic-devel https://github.com/AltziTS007/talos_sim.git
    ```
2. Install any missing dependencies using `rosdep`:

    ```bash
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build the workspace:

    ```bash
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-8
    ```

4. Source the workspace:

    ```bash
    source devel/setup.bash
    ```

## Usage

### Launching the Simulation

To start the simulation go to your terminal and hit ```roslaunch talos_description spawn_robocup.launch``` and hit play to gazebo in the left corner.

### Launch gmapping

Next, we hit gmapping for mapping ```roslaunch talos_navigation gmapping.launch```.

Move the robot with ```rosrun teleop_twist_keyboard teleop_twist_keyboard.py```.

### Save the map

Now we save the map by hitting ```rosrun map_server map_saver```.


## Controlling the Robot
(Under devel)

## Available Launch Files
(Under devel)

### 4. Conclusion

The simulation is in an early stage a lot of things need to be optimized from navigation, manipulation, state-machine, and of course vision. 


## Contributing

Contributions are welcome! If you would like to contribute to this project, please follow these steps:

1. Fork the repository.
2. Create a new branch (git checkout -b feature-branch).
3. Make your changes.
4. Commit your changes (git commit -am 'Add new feature').
5. Push to the branch (git push origin feature-branch).
6. Create a new Pull Request.

Please ensure your code adheres to the project's coding standards and includes appropriate tests.

:warning: *Every bug you encounter, address it to the Issues section above.* :warning:

Thank you!

## License
This project is licensed under the MIT License. See the LICENSE file for more details.

## Acknowledgements
This project is developed and maintained by AltziTS007. Special thanks to the contributors and the ROS community for their support and contributions.

### Sources from GitHub:

- https://github.com/iralabdisco/ira_laser_tools
- https://github.com/ros-perception/slam_gmapping
- https://github.com/ros-planning/navigation
- https://github.com/pal-robotics/realsense_gazebo_plugin
- https://github.com/issaiass/realsense2_description
- https://github.com/FlexBE/flexbe_app
- https://github.com/robocup-at-work/atwork-commander
