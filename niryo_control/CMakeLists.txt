cmake_minimum_required(VERSION 3.0.2)
project(niryo_control)
add_compile_options(-std=c++17 -Ofast)



set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    roscpp
    tf
    tf_conversions
    nav_msgs
    rci_qp_controller
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_action_files(DIRECTORY action FILES target.action control.action q_ctrl.action JointPostureCtrl.action SE3Ctrl.action)


generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs actionlib_msgs)

catkin_package(  
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS   
    sensor_msgs
    tf
    roscpp
    message_runtime 
    std_msgs
    tf_conversions
    rci_qp_controller
    
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rci_qp_controller_INCLUDE_DIRS}
)

install(PROGRAMS
	
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#SET(${PROJECT_NAME}_HEADERS
#)
#SET(${PROJECT_NAME}_SOURCES
#  src/controller/controller.cpp
#  src/server/JointPostureServer.cpp
#)


#ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS} )
#TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIR} )
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC eiquadprog::eiquadprog pinocchio::pinocchio -lpthread)

add_executable(${PROJECT_NAME}_node src/control.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} -lpthread)

TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}_node  SYSTEM PUBLIC $<INSTALL_INTERFACE:include>)

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
