cmake_minimum_required(VERSION 2.8.3)
project(squirrel_8dof_planner)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  tf
  sensor_msgs
  trajectory_msgs
  birrt_star_algorithm
  squirrel_motion_planner_msgs
  interactive_markers
  octomap_msgs
  octomap_server
  kdl_parser
  fcl
)

find_package(octomap REQUIRED)

find_package(orocos_kdl REQUIRED)


catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs nav_msgs geometry_msgs tf sensor_msgs trajectory_msgs birrt_star_algorithm interactive_markers octomap_msgs octomap_server  kdl_parser squirrel_motion_planner_msgs fcl
  DEPENDS octomap orocos_kdl
)
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OCTOMAP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
