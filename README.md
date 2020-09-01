# Ignition RViz

[RViz](http://wiki.ros.org/rviz) is a 3D visualization tool for robots using ROS.

Ignition RViz offers functionality similar to RViz, and is developed using
[Ignition](https://ignitionrobotics.org/) libraries.

![Ignition RViz CI](https://github.com/Sarath18/ign-rviz/workflows/Ignition%20RViz%20CI/badge.svg)

Head over to the [wiki](https://github.com/ignitionrobotics/ign-rviz/wiki) to get detailed description of the project.

### Requirements

- [ROS 2 Foxy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/)
- [Ignition Dome](https://ignitionrobotics.org/docs/dome) (under development)
	- [Ignition Common](https://ignitionrobotics.org/libs/common) 3
	- [Ignition GUI](https://ignitionrobotics.org/libs/gui) 4
	- [Ignition Math](https://ignitionrobotics.org/libs/math) 6
	- [Ignition Rendering](https://ignitionrobotics.org/libs/rendering) 4
- [Qt5](https://www.qt.io/)

### Setup the repository

- **Create a colcon workspace**
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
```
- **Clone the repository**
```bash
git clone https://github.com/ignitionrobotics/ign-rviz.git
```

- **Build ign-rviz**
```bash
# Go to the root of your colcon workspace
cd ../

# Source ROS2
source /opt/ros/foxy/setup.bash  # If using bash
source /opt/ros/foxy/setup.zsh   # If using zsh

# Build ign-rviz
colcon build
```

### Launch ign-rviz

Ignition Rviz can be launched using the following command

```bash
# Source the workspace
source install/setup.zsh

# Launch ign-rviz
ros2 launch ign_rviz rviz.launch.py
```

### Instruction to generate documentation

Project documentation can be generated with the help of doxygen using the following commands.

```bash
cd ~/colcon_ws/src/ign-rviz/docs/

# Generate documentation
doxygen rviz.doxyfile

# View documentation
firefox ./html/index.html
```

### Instruction to use plugins
- **ImageDisplay plugin**
    1. Launch a `sensor_msgs/msg/Image` publisher. For example
       ```bash
       # Source ROS2
       ros2 run image_tools cam2image
       ```
    2. Launch ign-rviz as mention above
    3. Load an ImageDisplay plugin and select the published topic.
