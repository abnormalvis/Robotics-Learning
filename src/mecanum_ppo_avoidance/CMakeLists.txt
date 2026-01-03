cmake_minimum_required(VERSION 3.10)
project(mecanum_ppo_avoidance)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    sensor_msgs
    roslint
    control_msgs
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
    std_msgs
    pluginlib
    forward_command_controller
    geometry_msgs
    tf
    nav_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    geometry_msgs
    base_local_planner
    costmap_2d
    nav_core
    )

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

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
    INCLUDE_DIRS
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    sensor_msgs
    roslint
    control_msgs
    control_toolbox
    controller_interface
    hardware_interface
    realtime_tools
    std_msgs
    pluginlib
    forward_command_controller
    geometry_msgs
    tf
    nav_msgs
    tf2_ros
    visualization_msgs
    geometry_msgs
    base_local_planner
    costmap_2d
    nav_core
    DEPENDS
    ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of Eigen3_INCLUDE_DIRS.
    ## Therefore, the DEPEND does not work as expected and we need to add the directory to the INCLUDE_DIRS
    # Eigen3

    ## Boost is not part of the DEPENDS since it is only used in source files,
    ## Dependees do not depend on Boost when they depend on this package.
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# generate_dynamic_reconfigure_options(
#     cfg/PathFollowLocalPlanner.cfg
# )
add_executable(teleop src/teleop.cpp)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    # Set manually because Eigen sets a non standard INCLUDE DIR
)

## Declare cpp executables
add_library(${PROJECT_NAME} SHARED
    src/mecanum_chassis_controller.cpp
    src/waypoint_global_planner.cpp
    src/path_follow_local_planner.cpp
)

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

target_link_libraries(
    teleop
    ${catkin_LIBRARIES}
)

install(
    DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING
    PATTERN "*.h"   # 找到头文件
    PATTERN "*.hpp"
)
install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION $(CATKIN_PACKAGE_LIB_DESTINATION)
    LIBRARY DESTINATION $(CATKIN_PACKAGE_LIB_DESTINATION)
)
##########################

roslint_cpp()