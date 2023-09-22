cmake_minimum_required(VERSION 2.8.3)
project(evaluation)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#set variable CATKIN_DEPENDENCIES
set (CATKIN_DEPENDENCIES
    roscpp
    message_generation
    tf2_ros
    tf2_geometry_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    eigen_conversions
    custom_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})

find_package(PCL REQUIRED)
find_package(Open3D REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
   DEPENDS PCL Open3D
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Open3D_INCLUDE_DIRS}
)

# message(WARNING "Using custom cv bridge with OpenCV ${Open3D_ROOT}")

add_executable(gt_extractor src/extract_gt.cpp)
target_link_libraries(gt_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Open3D_LIBRARIES})