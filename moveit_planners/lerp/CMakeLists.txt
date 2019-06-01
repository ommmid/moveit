cmake_minimum_required(VERSION 2.8.12)
project(moveit_planners_lerp)

add_compile_options(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED
    COMPONENTS
      interactive_markers
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      moveit_ros_perception
      pluginlib
      geometric_shapes
      pcl_ros
      pcl_conversions
      rosbag
      tf2_ros
      tf2_eigen
      tf2_geometry_msgs
)

catkin_package(
    LIBRARIES
      interactivity_utils
    INCLUDE_DIRS
    CATKIN_DEPENDS
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      interactive_markers
      tf2_geometry_msgs
    DEPENDS
      EIGEN3
)

# The following include_directory should have include folder of the new planner.
include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${TrajOpt_INCLUDE_DIRS})

# LERP planning plugin
add_library(moveit_lerp_planner_plugin src/lerp_planner_manager.cpp)
set_target_properties(moveit_lerp_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

add_library(model_based_planning_context src/model_based_planning_context.cpp)
set_target_properties(model_based_planning_context PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

add_executable(test_lerp src/test_lerp.cpp)
target_link_libraries(test_lerp ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  moveit_lerp_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES lerp_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



