cmake_minimum_required(VERSION 2.8.3)
project(cgr_gridmap_slam_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
        pcl_conversions
        pcl_ros
        mrpt_bridge
  openslam_gmapping
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(MRPT REQUIRED base obs gui slam)
find_package(Eigen3 REQUIRED)




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS roscpp tf std_msgs nav_msgs sensor_msgs openslam_gmapping mrpt_bridge
        LIBRARIES cgr_slam2d_processor
        INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include/cgr_gridmap_slam_ros ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


set(CGR_SLAM_LIB_FILES
        src/cgr_slam2d_processor.cpp
        src/motion_model.cpp
        src/cgr_scan_matcher.cpp)

# For clion
set(SOURCE_FILES
        nodes/slam_ros_wrapper.cpp
        nodes/slam_node.cpp
        ${CGR_SLAM_LIB_FILES}
        include/cgr_gridmap_slam_ros/cgr_slam2d_processor.h
        include/cgr_gridmap_slam_ros/slam_ros_wrapper.h
        )




add_library(cgr_slam2d_processor src/motion_model.cpp src/cgr_scan_matcher.cpp src/cgr_slam2d_processor.cpp include/cgr_gridmap_slam_ros/impl/icp.hpp)
target_link_libraries(cgr_slam2d_processor ${catkin_LIBRARIES} ${MRPT_LIBS} ${PCL_LIBRARIES})

add_executable(cgr_gridmap_slam_node nodes/slam_ros_wrapper.cpp nodes/slam_node.cpp)
target_link_libraries(cgr_gridmap_slam_node cgr_slam2d_processor ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install( TARGETS
        cgr_slam2d_processor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE
        )



