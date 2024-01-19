cmake_minimum_required(VERSION 3.12)
project(depth_image_averaging)
cmake_policy(SET CMP0148 OLD) # to surpress warnings. run: cmake --help-policy CMP0148

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  nodelet
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depth_image_averaging_nodelet
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs tf2_ros nodelet
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(depth_image_averaging_nodelet src/depth_image_averaging_nodelet.cpp)
target_link_libraries(depth_image_averaging_nodelet ${catkin_LIBRARIES})

add_executable(depth_image_averaging_node src/depth_image_averaging_node.cpp)
target_link_libraries(depth_image_averaging_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS depth_image_averaging_node depth_image_averaging_nodelet
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)