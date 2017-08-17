cmake_minimum_required(VERSION 2.8.3)
project(move_base_fault_tolerant)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        pluginlib
        actionlib
        move_base
        dynamic_reconfigure
        message_generation
        move_base_msgs
        nav_core
        fault_core
        tf
)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})
find_library(move_base move_base)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        tf
        move_base
        fault_core
        dynamic_reconfigure
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

# move_base_fault_tolerant
add_library(move_base_fault_tolerant
  src/move_base_fault_tolerant.cpp

)
target_link_libraries(move_base_fault_tolerant
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${move_base}
    )
add_dependencies(move_base_fault_tolerant move_base_fault_tolerant_gencfg)
add_dependencies(move_base_fault_tolerant geometry_msgs_gencpp)

add_executable(move_base_fault_tolerant_node
  src/move_base_fault_tolerant_node.cpp
)
target_link_libraries(move_base_fault_tolerant_node move_base_fault_tolerant)
set_target_properties(move_base_fault_tolerant_node PROPERTIES OUTPUT_NAME move_base_fault_tolerant)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )

install(
    TARGETS
        move_base_fault_tolerant
        move_base_fault_tolerant_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
