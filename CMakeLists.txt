cmake_minimum_required(VERSION 2.8.3)
project(lar_bonmet)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
)

#EIGEN
find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

#OpenMP
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

#Lar Tools
MESSAGE("#### LAR TOOLS #####")
find_package(lar_tools)
include_directories(${lar_tools_INCLUDE_DIRS})

##########################################################################

#Includes
include_directories(
   ${catkin_INCLUDE_DIRS}
  include/
)



#Packaging
catkin_package(
  INCLUDE_DIRS
    include/
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


#LAR VISION COMMONS LIBRARY
include_directories( include/bonmet)
add_library(lar_bonmet
        src/bonmet/BonmetInterface.cpp
)
target_link_libraries (lar_bonmet ${catkin_LIBRARIES} lar_tools)



#NODES

# planar_grasp_test
add_executable(${PROJECT_NAME}_keyboard_teleop  src/nodes/lar_bonment_keyboard_teleop.cpp)
target_link_libraries(${PROJECT_NAME}_keyboard_teleop ${catkin_LIBRARIES} lar_bonmet lar_tools)
