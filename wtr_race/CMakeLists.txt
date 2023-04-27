cmake_minimum_required(VERSION 2.8.3)

project(wtr_race)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  visualization_msgs
  geometry_msgs
  std_msgs
  tf
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  nav_core 
  pluginlib
  cv_bridge
  image_transport
  sensor_msgs
)

include_directories(
  include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/wtr_race/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/astar/
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ../include
)

add_message_files(
  FILES
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
 CATKIN_DEPENDS roscpp std_msgs message_runtime nav_core pluginlib
#  DEPENDS system_lib
)

add_library(Astar_planner_lib
  include/astar/astar.h
  src/astar/astar.cpp
)


add_executable(traj_plan_3D src/traj_plan_3D.cpp)
add_executable(test src/test.cpp)
add_executable(loop_dect  src/visual_dect/loop_dect.cpp)
add_executable(base2camera  src/tf/base2camera.cpp)

# add_dependencies(traj_plan_3D ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(traj_plan_3D ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Astar_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(loop_dect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(
  ${catkin_LIB_DIRS}
)
target_link_libraries(Astar_planner_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(traj_plan_3D
  Astar_planner_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(test
  ${catkin_LIBRARIES}
)
target_link_libraries(loop_dect
  ${catkin_LIBRARIES}
  # ${OpenCV_LIBS}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(base2camera
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/ros/video_tackle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 

