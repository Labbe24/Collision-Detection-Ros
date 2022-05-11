# Collision-Detection-Ros

## Description

## Setup
### Install prereqs
```
sudo apt install python3-colcon-common-extensions python3-vcstool
```
### Create colcon workspace
```
export COLCON_WS=~/workspace/colcon-ws
mkdir -p $COLCON_WS/src
cd $COLCON_WS
```

### Add Collision-detection package
```
git clone https://github.com/Labbe24/Collision-Detection-Ros src/Collision-Detection-Ros
```
### Add UR ROS2 driver

https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#getting-started step 4. -
```
git clone -b galactic https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.galactic.repos
```

### Add LBR FRI ROS2 Stack (Kuka driver)
https://github.com/KCL-BMEIS/lbr_fri_ros2_stack#first-steps

```
wget https://raw.githubusercontent.com/KCL-BMEIS/lbr_fri_ros2_stack/main/lbr_fri_ros2_stack/repos.yml -P src
vcs import src < src/repos.yml
```

### Build packages

```
rosdep update
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```