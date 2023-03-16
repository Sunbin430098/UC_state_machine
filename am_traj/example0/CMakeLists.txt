cmake_minimum_required(VERSION 2.8.3)

project(nav_traj_plan)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ../include
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_receiver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(example0_node src/example0_node.cpp)
add_executable(traj_plan src/traj_plan.cpp)
add_executable(traj_plan_3D src/traj_plan_3D.cpp)

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(example0_node
  ${catkin_LIBRARIES}
)
target_link_libraries(traj_plan
  ${catkin_LIBRARIES}
)
target_link_libraries(traj_plan_3D
  ${catkin_LIBRARIES}
)


