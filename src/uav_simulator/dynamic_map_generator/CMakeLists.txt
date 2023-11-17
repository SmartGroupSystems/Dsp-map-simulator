cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  pcl_conversions
  message_generation
  message_runtime
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  dynamic_obs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(click_map_dynamic src/click_map_dynamic.cpp)
add_dependencies(click_map_dynamic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(click_map_dynamic
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dyn_map src/dyn_map.cpp)
target_link_libraries(dyn_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

  add_executable(local_dyn_map src/local_dyn_map.cpp)
  target_link_libraries(local_dyn_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})