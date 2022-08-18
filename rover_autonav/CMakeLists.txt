cmake_minimum_required(VERSION 2.8.3)

project(rover_autonav)

## find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	nav_msgs
	visualization_msgs
	gazebo_ros
	gazebo_ros_control
)
## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS
	nav_msgs
	visualization_msgs
)
## Include Directories
include_directories(
	${catkin_INCLUDE_DIRS}
	${config}
	${launch}
	${meshes}
	${URDF}
	${nodes}
	${src}
	${world}
)

add_executable(rover_vis nodes/visualization.cpp)
target_link_libraries(rover_vis ${catkin_LIBRARIES})
add_dependencies(rover_vis ${catkin_EXPORTED_TARGETS})
