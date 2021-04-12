cmake_minimum_required(VERSION 3.5)
project(dsr_control2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fpermissive)
endif()

#??? find_package(ament_cmake REQUIRED COMPONENTS
#???   dsr_msgs2
#???   rclcpp
#??? )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dsr_msgs2 REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_msgs REQUIRED)


#??? catkin_package(
#???   INCLUDE_DIRS include
#???   CATKIN_DEPENDS dsr_msgs2 roscpp
#??? )


SET( COMMON_INC_FILES
  ../common2/include
)
SET( COMMON_LIB_FILES
  ../common2/lib
)

include_directories(
  include
  #ROS2 ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

  ${COMMON_INC_FILES}
)

SET( SRC_FILES
  #ROS2 ${SERIAL_SRC_FILES}
  src/dsr_hw_interface2.cpp
  src/dsr_control_node2.cpp
)

LINK_LIBRARIES(DRFL PocoFoundation PocoNet)

if($ENV{ROS_DISTRO} STREQUAL "foxy")                          
  LINK_DIRECTORIES ("${COMMON_LIB_FILES}/$ENV{ROS_DISTRO}/x86_64")
else()
  message("Unknown ROS distro:")
  message($ENV{ROS_DISTRO})
endif()

add_executable(dsr_control_node2 ${SRC_FILES})
#??? add_dependencies(dsr_control_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
ament_target_dependencies(dsr_control_node2 rclcpp std_msgs sensor_msgs dsr_msgs2 controller_manager hardware_interface moveit_msgs)

#target_link_libraries(dsr_control_node2 ${catkin_LIBRARIES})
target_link_libraries(dsr_control_node2 boost_system boost_thread)
target_link_libraries(dsr_control_node2 DRFL PocoFoundation PocoNet)


install(TARGETS dsr_control_node2
  #??? ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #??? LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #??? RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
   #??? DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
   #??? DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(dsr_msgs2)
ament_export_dependencies(controller_manager)
ament_export_dependencies(moveit_msgs)
ament_export_dependencies(hardware_interface)
ament_package()
