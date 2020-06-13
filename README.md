# Ignition RViz

RViz is a 3D visualization program for robots using ROS.

This version of RViz is developed using Ignition Rendering Library.

### Requirements

- ROS2
- ignition libraries
	- ign-common3
	- ign-gui4
	- ign-rendering4
	- ign-math6
- pluginlib
- Qt5

### Setup the repository

- **Create a colcon workspace**
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
```
- **Clone the repository**
```bash
git clone https://github.com/Sarath18/ign-rviz.git
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
