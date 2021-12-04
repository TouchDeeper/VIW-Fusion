cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport
        rosbag)
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)

# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})
#template sophus
#set(Sophus_DIR "/home/td/slam/install/sophus/share/sophus/cmake/")
#find_package(Sophus REQUIRED)
include_directories(/home/td/slam/install/sophus/include)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


catkin_package(
  INCLUDE_DIRS src
  LIBRARIES vins_lib
  CATKIN_DEPENDS roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport
        rosbag
)

add_library(vins_lib
        src/estimator/parameters.cpp
        src/estimator/estimator.cpp
        src/estimator/feature_manager.cpp
        src/factor/pose_local_parameterization.cpp
        src/factor/pose_subset_parameterization.cpp
        src/factor/orientation_local_parameterization.cpp
        src/factor/orientation_subset_parameterization.cpp
        src/factor/projectionTwoFrameOneCamFactor.cpp
        src/factor/projectionTwoFrameTwoCamFactor.cpp
        src/factor/projectionOneFrameTwoCamFactor.cpp
        src/factor/marginalization_factor.cpp
        src/utility/utility.cpp
        src/utility/visualization.cpp
        src/utility/CameraPoseVisualization.cpp
        src/initial/solve_5pts.cpp
        src/initial/initial_aligment.cpp
        src/initial/initial_sfm.cpp
        src/initial/initial_ex_rotation.cpp
        src/featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_executable(vins_node src/rosNodeTest.cpp)
target_link_libraries(vins_node vins_lib)

add_executable(viwo_node src/rosNodeTestWheel.cpp)
target_link_libraries(viwo_node vins_lib)

#add_executable(viwo_gazebo_node src/rosNodeTestWheelGazebo.cpp)
#target_link_libraries(viwo_gazebo_node vins_lib)

#add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
#target_link_libraries(kitti_odom_test vins_lib)

add_executable(simulation_node src/simulationTest.cpp)
target_link_libraries(simulation_node vins_lib)

#add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
#target_link_libraries(kitti_gps_test vins_lib)

add_executable(bag_writer src/tool/bagwriter.cpp)
target_link_libraries(bag_writer ${catkin_LIBRARIES})

