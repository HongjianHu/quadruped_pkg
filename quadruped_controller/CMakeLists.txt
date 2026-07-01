cmake_minimum_required(VERSION 3.8)
project(quadruped_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  std_msgs
  quadruped_controller_msgs
  pinocchio
  kdl_parser
)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
    src/QuadrupedController.cpp
    src/FSM/StatePassive.cpp
    src/FSM/StateFixedDown.cpp
    src/FSM/StateFreeStand.cpp
    src/FSM/StateTrotting.cpp
    src/FSM/StateFixedStand.cpp
    src/robot/QuadrupedRobot.cpp
    src/control/LowPassFilter.cpp
    src/control/Estimator.cpp
    src/control/BalanceCtrl.cpp
    src/gait/FeetEndCalc.cpp
    src/gait/GaitGenerator.cpp
    src/gait/WaveGenerator.cpp
    src/gait/Gait.cpp
    src/robot/go2_robot_data/PinGo2Model.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    PRIVATE
    src
)

ament_target_dependencies(
    ${PROJECT_NAME} PUBLIC
    ${CONTROLLER_INCLUDE_DEPENDS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  Eigen3::Eigen
  qpOASES
)

add_executable(gait_debug
    src/gait/GaitDebug.cpp
)

target_link_libraries(gait_debug
    PRIVATE
    ${PROJECT_NAME}
)

set_target_properties(gait_debug PROPERTIES
    INSTALL_RPATH "$ORIGIN"
)

pluginlib_export_plugin_description_file(controller_interface quadruped_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

install(
    TARGETS gait_debug
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
