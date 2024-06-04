cmake_minimum_required(VERSION 3.0.2)
project(octomap_converter)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_msgs
  octomap_ros
  tf2_ros
  nav_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES octomap_converter
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(octomap_limit_filter src/octomap_limit_filter.cpp)
target_link_libraries(octomap_limit_filter ${catkin_LIBRARIES})

add_executable(octomap_frame_converter src/octomap_frame_converter.cpp)
target_link_libraries(octomap_frame_converter ${catkin_LIBRARIES})

add_executable(octomap_to_occupancygrid src/octomap_to_occupancygrid.cpp)
target_link_libraries(octomap_to_occupancygrid ${catkin_LIBRARIES})

add_executable(octomap_speckle_remover src/octomap_speckle_remover.cpp)
target_link_libraries(octomap_speckle_remover ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS octomap_limit_filter octomap_frame_converter octomap_to_occupancygrid octomap_speckle_remover
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
