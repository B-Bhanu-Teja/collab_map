cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_clustering)

find_package(PCL 1.2 REQUIRED)
find_package(PkgConfig)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
visualization_msgs
sensor_msgs
geometry_msgs
message_generation
pcl_conversions
rviz_visual_tools
cv_bridge
pcl_ros
tf2
tf2_ros
tf2_sensor_msgs
)

add_message_files(
  FILES
  clusterInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs

)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES point_cloud_clustering
 CATKIN_DEPENDS roscpp message_runtime
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( ${PROJECT_NAME}
    src/cluster_extraction.cpp
)

# target_link_libraries( point_cloud_clustering
#     ${catkin_LIBRARIES} 
#     ${PCL_LIBRARIES}
#     ) 

add_executable( point_cloud_clustering_node
    src/point_cloud_clustering_node.cpp
)
target_link_libraries( point_cloud_clustering_node
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${PROJECT_NAME}
) 

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib
)