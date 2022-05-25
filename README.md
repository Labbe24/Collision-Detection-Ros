# Collision-Detection-Ros

## Description
This repository contains a launch file, launching a trajectory generation node, along with a TCP-endpoint, allowing the generated trajectories to be transfered to another application, such as the Collision-Detection application at: https://github.com/Labbe24/Collision-Detection.

### Trajectory generation interface

The trajectories are generated via. the "generate_trajectory_srv" and "generate_trajectory_srv1" services with the interface:
```
string move_group
sensor_msgs/JointState[] states
---
trajectory_msgs/JointTrajectory res
```

## Run with docker

Pull docker image:
```
docker pull jensnk/collision_detection_ros:galactic
```

Create & start docker container:
```
docker run --name docker_ros -it --network="host" jensnk/collision_detection_ros:galactic
```

Open terminal:
```
docker exec -it docker_ros bash
```

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

### Add packages
```
git clone https://github.com/Labbe24/Collision-Detection-Ros src/Collision-Detection-Ros
vcs import src < src/Collision-Detection-Ros/repos.yml
```

### Build packages

```
rosdep update
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```


## Launch nodes
```
cd ~/workspace/colcon-ws
source install/setup.bash
ros2 launch collision_detection_launch collision_detection_launch.launch.py
```