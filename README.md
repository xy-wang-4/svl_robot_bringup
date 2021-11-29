# This is a forked repo with some modifications for Autoware and F1tenth project
Assume svl and autoware have been installed and configured.

# Below is the original description

# SVL Robot Bringup

`svl_robot_bringup` is a ROS2 package to assist with using [Nav2](https://navigation.ros.org/), the ROS 2 navigation stack, with the SVL Simulator.

The repository contains:
-  A ROS 2 node named `odom_tf_node` that subscribes to the `/odom` topic from the simulator and broadcasts the `tf` transform between the `basefootprint` and `odom` frames for the Cloi robot.
- A launch file which launches `odom_tf_node` as well as the `lgsvl_bridge` and the `pointcloud_to_laserscan` node and broadcasts a few static transforms for the sensor frames.
- A map of the LG Seocho environment available in the SVL Simulator to be used for localization and planning.
- Params for running Nav2.
- An rviz config file.

**Note**: this repository is currently only supported on ROS 2 Foxy.

# Update

New components added:

-  A urdf file for F1tenth
-  A map for racing environment

# Instruction(localization)

sudo apt update

sudo apt install -y ros-foxy-navigation2 ros-foxy-nav2-bringup

cd ~/adehome

mkdir -p svl_test/src

cd svl_test/src

git clone https://github.com/xy-wang-4/svl_robot_bringup.git

cd ..

colcon build

source install/setup.bash

ros2 launch svl_robot_bringup robot_tf_launch.py

(New terminal)

ros2 launch nav2_bringup localization_launch.py params_file:=/path_to_this_package/svl_robot_bringup/params/nav2_params.yml map:=/path_to_this_package/svl_robot_bringup/maps/f1tenth.yaml

(New terminal)

/opt/lgsvl/simulator
then publish and start the f1tenth simulation

(New terminal)

ros2 run rviz2 rviz2 -d /path_to_this_package/svl_robot_bringup/rviz/nav2_cloi.rviz
set a intiall pose on rviz2

# SLAM(Mapping)

The simulated map was generated with [Nav2](https://github.com/SteveMacenski/slam_toolbox).
To generate your own map using F1tenth vehicles, follow the instructions.

# Instruction(Mapping)

mkdir -p svl_test/src

cd svl_test/src

git clone https://github.com/SteveMacenski/slam_toolbox

change line 4305 of Karto.h to following

'''

    m_NumberOfRangeReadings = static_cast<kt_int32u>(math::Round((GetMaximumAngle() -
      GetMinimumAngle()) /
      GetAngularResolution()) + residual) - 1;
      
'''

cd ..

colcon build

source install/setup.bash

ros2 launch svl_robot_bringup robot_tf_launch.py

(New terminal)

ros2 launch nav2_bringup localization_launch.py params_file:=/path_to_this_package/svl_robot_bringup/params/nav2_params.yml map:=/path_to_this_package/src/svl_robot_bringup/maps/f1tenth.yaml

(New terminal)

ros2 launch slam_toolbox online_async_launch.py params_file:=/path_to_this_package/svl_robot_bringup/params/mapper_params_online_async.yaml

(New terminal)

/opt/lgsvl/simulator
then publish and start the f1tenth simulation

(New terminal)

ros2 run rviz2 rviz2 -d /path_to_this_package/svl_robot_bringup/rviz/nav2_cloi.rviz

set a intiall pose on rviz2
