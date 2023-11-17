# Dsp-map-simulator
A uav simulator for dsp-map. If you have any questions, please ask in the "issues" section.

# Description
This repository mainly contains the following modules: 

(1) To construct dynamic and static maps, please refer to the section on [Building simulation maps](#building-simulation-maps). 

(2) To test [DSP-MAP](https://github.com/g-ch/DSP-map), please refer to the [Testing DSP-MAP](#testing-dsp-map). 

(3) To conduct drone planning simulations, please refer to the [Control in simulation](#control-simulation).


# Compile
__Tested environment__: Ubuntu 18.04 + ROS Melodic and Ubuntu 20.04 + ROS Noetic

This document includes DSP-MAP, therefore it requires some components of dsp-map. For the configuration of dsp-map, please refer [DSP-MAP](https://github.com/g-ch/DSP-map). If you do not wish to use dsp-map and only want to use the simulator, you can delete the dsp-map folder from the source package and compile without it.

1.Download and compile the repo

```
mkdir -p uav_simulator_ws/src
cd uav_simulator_ws/src
git clone https://github.com/SmartGroupSystems/Dsp-map-simulator
cd ..
catkin_make
```

2.Test

```
cd uav_simulator_ws
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator_example.launch
```

Then you can see a uav like this:
![simulator](fig/simulator.png)


# Building Simulation Maps
We have set up two different types of maps for testing. 

__Click-map__: You can use the two built-in components of rviz to click on the map to generate static and dynamic obstacles, respectively, like this:

```
cd uav_simulator_ws
source devel/setup.bash
cd src/Dsp-map-simulator/
./click_map.sh
```

![click_map](fig/click_map.gif)

You can use the following command to save the published map as a rosbag file.

```
./save_dyn_map.sh
```

__Random-map__: If you use this script, you will get an environment with randomly generated dynamic obstacles.

```
cd uav_simulator_ws
source devel/setup.bash
cd src/Dsp-map-simulator/
./random_map.sh
```


![random_map](fig/random_map.gif)

If you want to adjust the number and generation range of obstacles, please modify the corresponding launch file in the script file.

```
    <launch>
    <node pkg ="map_generator" name ="dyn_map" type ="dyn_map" output = "screen">
        <param name="map/x_size"   value="30.0"/>
        <param name="map/y_size"   value="30.0" />
        <param name="map/z_size"   value="8.0" />

        <param name="map/obs_num"  value="60"/>
        <param name="map/obs_traj" value="8.0" />
        <param name="map/w_l"      value="1.0" />
        <param name="map/h_l"      value="7.0" />
    </node>
 
</launch>
```

Please ensure that you have granted executable permissions to these two sh files before executing them.

# Testing DSP-MAP

To test DSP-MAP, execute the following code in a window:

```
cd uav_simulator_ws
source devel/setup.bash
cd src/Dsp-map-simulator/
./click_map.sh
```

Then in a new window:

```
cd ~/uav_simulator_ws/
source devel/setup.bash 
roslaunch dynamic_occpuancy_map mapping.launch 
```

Then you can obtain the following results:
![dsp_map](fig/dsp_map.gif)

It is important to note that the simulation does not take into account the issue of light projection, so the obstacles will not obstruct each other.

# Control simulation
This project also provides a control interface:

```
cd uav_simulator_ws
source devel/setup.bash
roslaunch so3_quadrotor_simulator simulator_example.launch
```

Then in a new window:
 
```
source devel/setup.bash
rosrun so3_control control_keyboard 
```

Then you can use your keyboard to control the uav. Use 'WASD' to control the UAV's up, down, and yaw angle, and use 'IJKL' to control the UAV's forward, backward, left, and right movements.
![control](fig/control.gif)

If you want to customize the control rate, please refer to the ```control_keyboard.cpp``` interface that we have provided.