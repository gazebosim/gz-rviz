# Ignition RViz

[RViz](http://wiki.ros.org/rviz) is a 3D visualization tool for robots using ROS.

Ignition RViz offers functionality similar to RViz, and is developed using
[Ignition](https://ignitionrobotics.org/) libraries.

### Build Status

* ROS versions: Foxy and Rolling
* Ignition versions: Dome and Edifice

[![Ignition RViz CI](https://github.com/ignitionrobotics/ign-rviz/actions/workflows/ci.yml/badge.svg)](https://github.com/ignitionrobotics/ign-rviz/actions/workflows/ci.yml)

Head over to the [wiki](https://github.com/ignitionrobotics/ign-rviz/wiki) to get detailed description of the project.

### Requirements

- [ROS 2 Foxy or Rolling](https://docs.ros.org/en/rolling/Releases.html)
- [Ignition Dome or Edifice](https://ignitionrobotics.org/docs/)
	- [Ignition Common](https://ignitionrobotics.org/libs/common)
	- [Ignition GUI](https://ignitionrobotics.org/libs/gui)
	- [Ignition Math](https://ignitionrobotics.org/libs/math)
	- [Ignition Rendering](https://ignitionrobotics.org/libs/rendering)
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
git clone https://github.com/ignitionrobotics/ign-rviz.git
```

**Choose an Ignition version**

```bash
export IGNITION_VERSION=<ignition>
```

Where `<ignition>` can be `dome` or `edifice`. Defaults to Edifice if not set.

> You only need to set this variable when compiling, not when running.

**Build ign-rviz**

Go to the root of your colcon workspace

```bash
cd ../
```

Source ROS2, where `<distro>` is `foxy` or `rolling`:

```bash
source /opt/ros/<distro>/setup.bash  # If using bash
source /opt/ros/<distro>/setup.zsh   # If using zsh
```

Build ign-rviz

```bash
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
    2. Launch ign-rviz as mentioned above
    3. Load an `ImageDisplay` plugin and select the published topic.
