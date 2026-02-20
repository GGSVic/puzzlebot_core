# Puzzlebot Core

This repository includes the description and bringup packages for the **Puzzlebot** from Manchester Robotics. It is important to emphasize that this project only contains the setup for a simulation environment. It is highly recommended to treat these packages as a module/repo since they are included in multiple projects.

![Puzzlebot Simulation](assets/image.png)

---

## Prerequisites
- **ROS 2:** Humble  
- **Simulator:** Gazebo Ignition Fortress  

---

## Installation
Installing this package is quite simple. You only need to clone this repository into your workspace and check the corresponding ROS dependencies:

```sh
# Go to your workspace src folder
cd ~/ros2_ws/src

# Clone the repository
git clone  https://github.com/GGSVic/puzzlebot_core.git puzzlebot
# Change to ignition branch
cd puzzlebot
git checkout ign-sim
# Install dependencies and build
cd ../../
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros_gz_puzzlebot_description ros_gz_puzzlebot_bringup
```
---
### How to use

The bringup package includes three launch files: 

* `state_publisher.launch.py`:  Launches the ***robot_state_publisher*** node with the urdf file as an argument and its corresponding ***RVIZ*** visualization. 
* `spawn_puzzlebot.launch.py`: Reusable launch file that allows the user to spawn a puzzlebot within a Gazebo world simulation using the ***spawn_entity.py*** node. 
* `minimal_simulation.launch.py`: Designed to test that everything is set up correctly. It launches an empty world and includes the launch files already described. 

After sourcing the workspace where the packages are located, an important environment variable will appear in your system: ***PUZZLEBOT_MODEL***. You can change between three types of models:

- `drive`: The simplest version. It includes a basic chassis and the corresponding wheels and plugins to move the robot.

- `vision`: Expands the drive model by adding a functional camera.

- `perception`: Includes both the camera and a LiDAR sensor for more advanced projects.

Depending on the model you want to use, you will need to set the variable properly (set as drive by default):

``` sh
# Go to your workspace src folder
cd ~/ros2_ws/src
# Source the workspace
source install/setup.bash
export PUZZLEBOT_MODEL=drive # Options: drive, vision, perception
ros2 launch ros_gz_puzzlebot_bringup minimal_simulation.launch.py
```


---
## Credits
The robot is provided by [Manchester Robotics]( https://github.com/ManchesterRoboticsLtd), and the used meshes were originally extracted from its repository [puzzlebot_ros](https://github.com/ManchesterRoboticsLtd/puzzlebot_ros.git).

---
## License

Apache-2.0