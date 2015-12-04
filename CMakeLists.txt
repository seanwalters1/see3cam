cmake_minimum_required(VERSION 2.8.3)
project(see3cam)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure camera_info_manager image_transport nodelet roscpp sensor_msgs)

find_package(Boost REQUIRED COMPONENTS thread system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(udev REQUIRED libudev)

generate_dynamic_reconfigure_options(cfg/see3cam.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nodelet_see3cam
  CATKIN_DEPENDS roscpp dynamic_reconfigure nodelet sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(xunit
  src/xunit/xunit_lib.c
  src/xunit/xunit_lib_cug.c
)
target_link_libraries(xunit ${udev_LIBRARIES})

add_library(see3cam_nodelet src/nodelets.cpp src/stereo.cpp src/uvc_cam.cpp)
target_link_libraries(see3cam_nodelet ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunit)

add_executable(see3cam_stereo_node src/stereo_node.cpp src/stereo.cpp src/uvc_cam.cpp)
target_link_libraries(see3cam_stereo_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunit)

add_dependencies(see3cam_stereo_node ${PROJECT_NAME}_gencfg)
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS see3cam_stereo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
