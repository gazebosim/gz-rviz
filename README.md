# Gazebo RViz

[RViz](http://wiki.ros.org/rviz) is a 3D visualization tool for robots using ROS.

Gazebo RViz offers functionality similar to RViz, and is developed using
[Gazebo](https://gazebosim.org/) libraries.

### Build Status

* ROS versions: Rolling (Humble)
* Gazebo versions: Fortress or Garden

[![Gazebo RViz CI](https://github.com/gazebosim/gz-rviz/actions/workflows/ci.yml/badge.svg)](https://github.com/gazebosim/gz-rviz/actions/workflows/ci.yml)

Head over to the [wiki](https://github.com/gazebosim/gz-rviz/wiki) to get detailed description of the project.

### Requirements

- [ROS 2](https://docs.ros.org/en/rolling/Releases.html)
- [Gazebo](https://gazebosim.org/docs/)
	- [Gazebo Common](https://gazebosim.org/libs/common)
	- [Gazebo GUI](https://gazebosim.org/libs/gui)
	- [Gazebo Math](https://gazebosim.org/libs/math)
	- [Gazebo Rendering](https://gazebosim.org/libs/rendering)
- [Qt5](https://www.qt.io/)
- Additional QML modules required for GPS Plugin
  - [QtPositioning](https://doc.qt.io/qt-5/qtpositioning-index.html) (`qml-module-qtpositioning`)
  - [QtLocation](https://doc.qt.io/qt-5/qtlocation-index.html) (`qml-module-qtlocation`)

### Setup the repository

**Create a colcon workspace**

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
```

**Clone the repository**

```bash
git clone https://github.com/gazebosim/gz-rviz.git
```

**Choose an Gazebo version**

```bash
export GZ_VERSION=<gazebo version>
```

Where `<gazebo>` should be `fortress` or `garden`.

> You only need to set this variable when compiling, not when running.

**Build gz-rviz**

Go to the root of your colcon workspace

```bash
cd ../
```

Source ROS2, where `<distro>` is `rolling`:

```bash
source /opt/ros/<distro>/setup.bash  # If using bash
source /opt/ros/<distro>/setup.zsh   # If using zsh
```

Build gz-rviz

```bash
colcon build
```

### Launch gz-rviz

Gazebo Rviz can be launched using the following command

```bash
# Source the workspace
source install/setup.zsh

# Launch gz-rviz
ros2 launch gz_rviz rviz.launch.py
```

### Instruction to generate documentation

Project documentation can be generated with the help of doxygen using the following commands.

```bash
cd ~/colcon_ws/src/gz-rviz/docs/

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
    2. Launch gz-rviz as mentioned above
    3. Load an `ImageDisplay` plugin and select the published topic.
