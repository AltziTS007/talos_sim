cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)
project(laser_odometry_rf2o)

find_package(catkin REQUIRED COMPONENTS
  laser_odometry_core rf2o_laser_odometry #rosparam_handler dynamic_reconfigure
)

#generate_ros_parameter_files(
#  cfg/LaserOdometryPolar.params
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS laser_odometry_core rf2o_laser_odometry #rosparam_handler dynamic_reconfigure
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

###############
##   Build   ##
###############

add_library(${PROJECT_NAME} src/laser_odometry_rf2o.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg) # For dynamic_reconfigure
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_genparam) # For rosparam_handler

#Require C++11
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_EXTENSIONS OFF)
endif()

################
##  Install   ##
################

install(TARGETS ${PROJECT_NAME}
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES laser_odometry_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

  install(DIRECTORY launch #cfg
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

################
##    Test    ##
################

#if(CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    find_package(catkin REQUIRED COMPONENTS
#        roslib
#    )

#    add_rostest_gtest(test_carmen_reader_test
#      test/stuff.test
#      test/stuff.cpp
#    )

#  target_link_libraries(test_csm_test ${catkin_LIBRARIES} ${PROJECT_NAME})

#endif()
