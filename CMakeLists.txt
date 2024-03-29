cmake_minimum_required(VERSION 3.12)
project(depth_image_averaging)
cmake_policy(SET CMP0148 OLD) # to surpress warnings. run: cmake --help-policy CMP0148

add_compile_options(-std=c++17)

set(USE_OPENCL 0) # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

add_compile_definitions(USE_OPENCL=${USE_OPENCL})

find_package(catkin REQUIRED COMPONENTS
  depth_image_proc
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  nodelet
  image_transport
)

if(USE_OPENCL)
find_package(OpenCL REQUIRED)
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES depth_image_averaging_nodelet
  CATKIN_DEPENDS depth_image_proc roscpp sensor_msgs std_msgs tf2_ros nodelet image_transport
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(depth_image_averaging_nodelet src/depth_image_averaging_nodelet.cpp src/depth_image_averager.cpp)

if(USE_OPENCL)
target_link_libraries(depth_image_averaging_nodelet ${catkin_LIBRARIES} OpenCL::OpenCL)
else()
target_link_libraries(depth_image_averaging_nodelet ${catkin_LIBRARIES})
endif()

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

file(GLOB KERNELS src/opencl/*.cl)
foreach(CurrentKernel IN LISTS KERNELS)
    cmake_path(GET CurrentKernel FILENAME FileName)
    add_custom_command(
                TARGET depth_image_averaging_node PRE_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CurrentKernel} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/opencl/${FileName}
                COMMENT "Copying kernel: ${CurrenKernel}")
endforeach()