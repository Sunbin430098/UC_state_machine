cmake_minimum_required(VERSION 2.8.3)

project(wtr_race)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
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
)

include_directories(
  include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/wtr_race/
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
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
 CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

add_executable(traj_plan_3D src/traj_plan_3D.cpp)

# add_dependencies(traj_plan_3D ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(traj_plan_3D ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

link_directories(
  ${catkin_LIB_DIRS}
)
target_link_libraries(traj_plan_3D
  ${catkin_LIBRARIES}
)

