cmake_minimum_required(VERSION 2.8.3)
project(simple_recovery_behavior)

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        pluginlib
        nav_core
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
        nav_core
)

add_library(simple_recovery_behavior ros/src/simple_recovery_behavior.cpp)
target_link_libraries(simple_recovery_behavior ${catkin_LIBRARIES})

### INSTALLS
install(TARGETS simple_recovery_behavior
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES recovery_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
