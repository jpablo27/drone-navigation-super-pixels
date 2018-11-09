cmake_minimum_required(VERSION 2.8.3)
project(sdk)


if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  pcl_msgs
  pcl_ros
  message_generation
	tf2_geometry_msgs
	mavros
	mavros_msgs
  )

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZED 2 REQUIRED)

add_message_files(
  FILES
  drone_MSG.msg
  drone_colors.msg
  drone_ver.msg
  e_msgs.msg
  c_msgs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_msgs
)

catkin_package()


include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include include)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(include src)
add_subdirectory(ORUtils)

set(GSLICR_LIB
gSLICr_Lib/engines/gSLICr_core_engine.h
gSLICr_Lib/engines/gSLICr_seg_engine.h
gSLICr_Lib/engines/gSLICr_seg_engine_GPU.h
gSLICr_Lib/engines/gSLICr_seg_engine_shared.h
gSLICr_Lib/engines/gSLICr_core_engine.cpp
gSLICr_Lib/engines/gSLICr_seg_engine.cpp
gSLICr_Lib/engines/gSLICr_seg_engine_GPU.cu
gSLICr_Lib/objects/gSLICr_settings.h
gSLICr_Lib/objects/gSLICr_spixel_info.h
gSLICr_Lib/gSLICr_defines.h
gSLICr_Lib/gSLICr.h
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${ZED_LIBRARY_DIR})

list(APPEND "-std=c++11 -ftree-vectorize")
SOURCE_GROUP(engines FILES ${GSLICR_LIB})

cuda_add_library(gSLICr_lib
			${GSLICR_LIB}
			src/NVTimer.h
			OPTIONS -gencode arch=compute_30,code=compute_30)

target_link_libraries(gSLICr_lib ${CUDA_LIBRARY})

add_definitions(${PCL_DEFINITIONS} -std=c++11 -g -O3)

add_executable(drone_node src/main.cpp
src/sp1_callbacks.cpp
include/colours.cpp
include/connections.cpp
include/coordinates.cpp
include/fill.cpp
include/fillpcl.cpp
include/gpoly.cpp
include/planex.cpp
include/adj_mat_mask.cpp
include/connected_components_listv.cpp
include/connected_components_listLAB.cpp
include/connected_components_listDepth.cpp
include/ar_polygons.cpp
include/sendHulls.cpp
include/rrt.cpp
)


target_link_libraries(drone_node
	gSLICr_lib
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${ZED_LIBRARIES}
)


