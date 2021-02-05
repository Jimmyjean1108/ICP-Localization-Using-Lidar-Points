cmake_minimum_required(VERSION 3.0.2)
project(localization_competition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization_competition
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (itri src/itri_icp.cpp)
target_link_libraries (itri ${PCL_LIBRARIES})
target_link_libraries (itri ${catkin_LIBRARIES})

add_executable (nuscenes1 src/nuScenes_icp.cpp src/map_segmentation.cpp)
target_link_libraries (nuscenes1 ${PCL_LIBRARIES})
target_link_libraries (nuscenes1 ${catkin_LIBRARIES})

add_executable (nuscenes2 src/nuScenes_2_icp.cpp src/map_segmentation.cpp)
target_link_libraries (nuscenes2 ${PCL_LIBRARIES})
target_link_libraries (nuscenes2 ${catkin_LIBRARIES})

