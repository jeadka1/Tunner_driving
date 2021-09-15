cmake_minimum_required(VERSION 2.8.3)
project(leo_driving)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        tf
        nodelet
        tf2_sensor_msgs
        pcl_conversions
        laser_geometry
	message_generation
	std_msgs
	actionlib_msgs
	geometry_msgs
	nav_msgs
	std_srvs
        )
find_package(PCL REQUIRED)
add_message_files(
  DIRECTORY msg
  FILES
  BoundingBox.msg
  InstanceSegment.msg
  DrainCoef.msg
  MissionItem.msg
  Mission.msg

  EnumLowMode.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
  ImageSize.srv
  MissionUpload.srv
  SetInt32.srv
  SetFloat32.srv
  SetFloat64.srv
  Mode.srv
  SetBool.srv
  WaypointDistance.srv
  VideoStream.srv
  SetUint8.srv
  RobotPose.srv
  WaypointPath.srv
  charging_done.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  Waypoint.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
  sensor_msgs
  actionlib_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        LIBRARIES LocalizationNode AisleDetectNode ObstaclesNode CmdPublishNode ArucoNode MaincommandNode
        CATKIN_DEPENDS nodelet roscpp sensor_msgs tf geometry_msgs fiducial_msgs pcl_conversions laser_geometry message_runtime
        DEPENDS PCL
)

#add_executable(rp_driving_node src/rp_node.cpp )
#target_link_libraries(rp_driving_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(LocalizationNode src/localization_node.cpp)
add_dependencies(LocalizationNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(LocalizationNode ${catkin_LIBRARIES})

add_library(AisleDetectNode src/aisle_detect_node.cpp)
add_dependencies(AisleDetectNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AisleDetectNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(ObstaclesNode src/obstacles_node.cpp)
add_dependencies(ObstaclesNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ObstaclesNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(CmdPublishNode src/cmd_publish_node.cpp)
add_dependencies(ObstaclesNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ObstaclesNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(ArucoNode src/aruco_node.cpp)
add_dependencies(ArucoNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ArucoNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(MaincommandNode src/main_command_node.cpp)
add_dependencies(MaincommandNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MaincommandNode ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_library(${PROJECT_NAME}_nodelets
  src/localization_node.cpp
  src/aisle_detect_node.cpp
  src/obstacles_node.cpp
  src/cmd_publish_node.cpp
  src/aruco_node.cpp
  src/main_command_node.cpp)

target_link_libraries(${PROJECT_NAME}_nodelets LocalizationNode AisleDetectNode ObstaclesNode CmdPublishNode ArucoNode MaincommandNode)


install(TARGETS LocalizationNode AisleDetectNode ObstaclesNode CmdPublishNode ArucoNode MaincommandNode
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## msg
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  PATTERN ".svn" EXCLUDE
)

## srv
install(DIRECTORY srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
  PATTERN ".svn" EXCLUDE
)

## action
install(DIRECTORY action/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/action
  PATTERN ".svn" EXCLUDE
)


