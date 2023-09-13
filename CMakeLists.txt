cmake_minimum_required(VERSION 3.5)
project(my_vins)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)
# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
# find_package(Pangolin REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR} 
  ${CERES_INCLUDE_DIRS}
  # ${Pangolin_INCLUDE_DIRS}
  )

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR/lib})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(my_vins_lib SHARED
  src/estimator/parameters.cpp
  src/estimator/estimator.cpp
  src/estimator/feature_manager.cpp
  # src/factor/pose_local_parameterization.cpp
  # src/factor/projectionTwoFrameOneCamFactor.cpp
  # src/factor/projectionTwoFrameTwoCamFactor.cpp
  # src/factor/projectionOneFrameTwoCamFactor.cpp
  # src/factor/marginalization_factor.cpp
  src/utility/utility.cpp
  # src/utility/visualization.cpp
  # src/utility/CameraPoseVisualization.cpp
  # src/initial/solve_5pts.cpp
  # src/initial/initial_aligment.cpp
  # src/initial/initial_sfm.cpp
  # src/initial/initial_ex_rotation.cpp
  src/featureTracker/feature_tracker.cpp)
target_link_libraries(my_vins_lib ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(vins_test src/vins_test.cpp)
target_link_libraries(vins_test my_vins_lib) 

# add_executable(vins_node src/rosNodeTest.cpp)
# target_link_libraries(vins_node vins_lib) 

# add_executable(kitti_odom_test src/KITTIOdomTest.cpp)
# target_link_libraries(kitti_odom_test vins_lib) 

# add_executable(kitti_gps_test src/KITTIGPSTest.cpp)
# target_link_libraries(kitti_gps_test vins_lib) 

