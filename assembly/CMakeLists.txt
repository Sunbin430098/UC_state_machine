cmake_minimum_required(VERSION 2.8.3)

project(assembly2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  rospy
  tf
)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

catkin_package()

find_package(roslaunch)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS} lib
  ${GAZEBO_LIBRARY_DIRS}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_library(my_joint_controller SHARED src/joint_controller.cc)
target_link_libraries(my_joint_controller 
${catkin_LIBRARIES} 
${GAZEBO_LIBRARIES}
)
