cmake_minimum_required(VERSION 2.8.3)
project(tag_pose_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME}
   src/tag_pose_controller.cpp
   src/PID.cpp
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )
