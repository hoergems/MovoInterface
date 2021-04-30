find_package(catkin REQUIRED roscpp actionlib_msgs actionlib manipulator_utils movo_msgs)

set (TRANSITION_PLUGIN_SRC
     ${CMAKE_CURRENT_LIST_DIR}/MovoTransitionPlugin.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MovoRobotInterface.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MovoMobileBase.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/RobotiqInterface.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/HardwareStartup.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MovoAPI.cpp
     ${ROOT_PATH}/plugins/shared/MovoRobotInterface/MotionPlanner.cpp
     ${ROOT_PATH}/plugins/shared/RRTConnect/RRTConnect.cpp
     ${ROOT_PATH}/plugins/shared/RRTConnect/Tree.cpp)

ADD_TRANSITION_PLUGIN(movoTransitionPlugin "${TRANSITION_PLUGIN_SRC}")
target_include_directories(movoTransitionPlugin 
                           PRIVATE 
                           ${ROOT_PATH}/plugins/shared/
                           ${catkin_INCLUDE_DIRS})
#target_link_libraries(movoTransitionPlugin PUBLIC catkin)
