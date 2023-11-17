roslaunch so3_quadrotor_simulator simulator_example.launch & sleep 2;
rosrun map_generator click_map_dynamic & sleep 2;
#roslaunch map_generator dyn_map.launch & sleep 3;
rosrun map_generator local_dyn_map
