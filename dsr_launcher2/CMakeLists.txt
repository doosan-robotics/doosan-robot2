cmake_minimum_required(VERSION 3.5)
project(dsr_launcher2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#ROS2 find_package(catkin REQUIRED COMPONENTS roslaunch)
find_package(ament_cmake REQUIRED)

#ROS2 catkin_package()
ament_package()

install(DIRECTORY
	launch rviz worlds
	#ROS2 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	DESTINATION share/${PROJECT_NAME} 
)
