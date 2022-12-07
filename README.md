## ros2_roomba
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
---
# Redirection from obstacle

### Overview

This ROS Package implements a primitive obstacle avoidance tactic using Lidar Scan data.

### Dependencies/ Assumptions

#### Dependencies:
- OS : Ubuntu 22.04 
- ROS2 Distro : ROS2 Humble
- Package build type : ```ament_cmake ```
- Package dependencies : ```rclcpp```, ```std_msgs``` 
- ROS2 Humble Installation : [link](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- Turtlebot3 Simulation for ros2: [link](https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2)

#### Assumptions:
This package assumes that the turtlebot3-simulation package is installed as a part of the ROS workspace

## Instructions to run the ROS Package
### Build
Open a terminal
```
cd <your_ROS2_ws>/src
git clone https://github.com/aniruddhbalram97/ros2_roomba.git
cd .. 

# check for any dependencies
rosdep install -i --from-path src --rosdistro humble -y

# choose the turtlebot3 model
export GAZEBO_MODEL_PATH=`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
export TURTLEBOT3_MODEL=waffle_pi

# build both turtlebot3_simulation and ros2_roomba
colcon build --packages-select ros2_roomba
colcon build --packages-select turtlebot3_simulations

# source your installation
source . install/setup.bash

```

### Run Launch File to run simulation
To run the Launch file for obstacle avoidance node,run:
```

ros2 launch ros2_roomba turtlebot3_world.launch.py
```

### Edit the ros_bag.launch for different topics

To enable/disable rosbags, go to file ros_bag.launch.py and change "all_topics". (set to true by default)

To run them separately, you can use:
```
ros2 bag record -o saved_bag <topic>
```
For topic information in bag
```
ros2 bag info saved_bag
```
Play back data from rosbag
```
ros2 bag play saved_bag
``` 
## Results
The results are kept in the results folder

### rqt Console
```
 ros2 run rqt_console rqt_console

```
### cppcheck
Run the following command from the root directory of your ROS package
```
cppcheck --enable=all --std=c++17 ./src/*.cpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
### cpplint
Run the following command from the root directory of your ROS package
```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp > ./results/cpplint.txt
```

### Note: 
Sometimes gazebo runs in the background (gz_client). So just run the following commands to quit it;
```
ps aux
# find the pid associated to gz_client
sudo kill -2 <pid_gz_client>
```