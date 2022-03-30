cmake_minimum_required(VERSION 3.5)
project(trajectory_edit_panel)

add_definitions("-g")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS std_msgs tf roscpp pcl_conversions rviz visualization_msgs roslib)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(tiffer "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui QtXml)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
  #qt4_wrap_cpp(QT_MOC include/ur_pad.h include/Mythread.h include/ur3_config.h)
else()
  message(tiffer "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets Multimedia Xml)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Core Qt5::Widgets Qt5::Multimedia Qt5::Xml )
  qt5_wrap_cpp(
                                 QT_MOC 
                                 include/TrajectoryEditPanel.h
                                 )
endif()

add_definitions(-DQT_NO_KEYWORDS)

link_directories(
    /opt/ros/melodic/lib/
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${ROS_DEVEL_LIB_DIR}
)

set(SOURCES 
src/TrajectoryEditPanel.cpp
${QT_MOC}
)

add_library(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES}
    # carm_api
    # com_center
    # carm_common
    # network_interface
    # PocoNet
    # pthread
    # water_api
    # cleanbot_api
    # tcp_com
    # jsoncpp
    # common
    # point_cloud_process
    # bcdbuilder
    # cmotion_interface_coverage_plan
    # cmotion_interface_move_group 
    # coverage_segmentation
    # almost_ompl_interface
    # planning_interface  
    # robot_kinematics
    # robot_model
    # robot_state 
    # utils       
    # almost_collision_detection_fcl
    # rviz_visual_tools
    # rviz_visual_tools_gui
    # rviz_visual_tools_imarker_simple
    # # CGAL::CGAL
    # # CGAL::CGAL_Core
    ${FCL_LIBRARIES}
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${OCTOMAP_LIBRARIES}
    ${GTEST_LIBRARIES}
)

# FILES ########################################