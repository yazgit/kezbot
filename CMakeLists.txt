cmake_minimum_required(VERSION 2.8.3)
project(kezbot_v2)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

set(RPLIDAR_SDK_PATH "./include/rplidar_sdk")


# TODO fix problem for making rplidar_ros node
FILE(GLOB RPLIDAR_SDK_SRC
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  joy
  sensor_msgs
  roscpp
  rosconsole
  rospy
  tf
  tf2
  turtlesim
  rosserial_arduino
  rosserial_client
)

include_directories(
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kezbot_v2
#  CATKIN_DEPENDS geometry_msgs joy roscpp rospy tf tf2 turtlesim
#  DEPENDS system_lib
)

# TODO implement arduiono-cmake here
# rosserial_generate_ros_lib(
# 	PACKAGE rosserial_arduino
# 	SCRIPT make_libraries.py
# )
#
# rosserial_configure_client(
# 	DIRECTORY firmware
# 	TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
# )
#
# rosserial_add_client_target(firmware main_firmware ALL)
# rosserial_add_client_target(firmware main_firmware-upload)

add_executable(kezbot_v2_sim_tf_broadcaster src/sim_tf_broadcaster.cpp)
target_link_libraries(kezbot_v2_sim_tf_broadcaster ${catkin_LIBRARIES})

add_executable(kezbot_v2_sim_odom_publisher src/sim_odom_publisher.cpp)
target_link_libraries(kezbot_v2_sim_odom_publisher ${catkin_LIBRARIES})




add_executable(kezbot_v2_rplidar_node src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(kezbot_v2_rplidar_node ${catkin_LIBRARIES})

add_executable(kezbot_v2_tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(kezbot_v2_tf_broadcaster ${catkin_LIBRARIES})

add_executable(kezbot_v2_odom_publisher src/odom_publisher.cpp)
target_link_libraries(kezbot_v2_odom_publisher ${catkin_LIBRARIES})

add_executable(kezbot_v2_teleop_joy src/teleop_joy.cpp)
target_link_libraries(kezbot_v2_teleop_joy ${catkin_LIBRARIES})
