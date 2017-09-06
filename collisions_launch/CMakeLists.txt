cmake_minimum_required(VERSION 1.8.3)
project(collisions_launch)

find_package(catkin REQUIRED rostest COMPONENTS)

catkin_package()

### TESTS
if(CATKIN_ENABLE_TESTING)
  add_rostest(ros/test/2dnav_test.launch)
endif()

### INSTALLS
install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)
