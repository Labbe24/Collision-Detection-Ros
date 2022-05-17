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

### Build packages

```
rosdep update
rosdep install --ignore-src --from-paths src -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```