cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_sample)

#find_package(catkin REQUIRED
#	turtlebot3_bringup
#)
find_package(catkin REQUIRED
	roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_sample
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
	${catkin_INCLUDE_DIRS}
)


add_executable(sample src/sample.cpp)
target_link_libraries(sample ${catkin_LIBRARIES})
