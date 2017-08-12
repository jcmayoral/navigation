cmake_minimum_required(VERSION 2.8.3)
project(simple_costmap_layer)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/ros/include)

add_library(simple_costmap ros/src/simple_costmap_layer.cpp)

### INSTALLS
install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
