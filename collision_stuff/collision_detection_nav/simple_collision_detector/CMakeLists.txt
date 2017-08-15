cmake_minimum_required(VERSION 2.8.3)
project(simple_collision_detector)

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        pluginlib
        fault_core
        )

include_directories(ros/include
                    ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS
      ros/include
    LIBRARIES simple_recovery_behavior
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
        pluginlib
        fault_core
)

add_library(simple_collision_detector ros/src/simple_collision_detector.cpp)
target_link_libraries(simple_collision_detector ${catkin_LIBRARIES})

### INSTALLS
install(TARGETS simple_collision_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES fault_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
