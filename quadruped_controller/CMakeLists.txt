cmake_minimum_required(VERSION 3.8)
project(quadruped_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

get_filename_component(QUADRUPED_PKG_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH
    "${QUADRUPED_PKG_ROOT}/third_party/osqp"
    "${QUADRUPED_PKG_ROOT}/third_party/osqp-eigen"
)
set(QUADRUPED_THIRD_PARTY_RPATH
    "$ORIGIN;${QUADRUPED_PKG_ROOT}/third_party/osqp/lib;${QUADRUPED_PKG_ROOT}/third_party/osqp-eigen/lib"
)

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
find_package(OsqpEigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
    src/QuadrupedController.cpp
    src/FSM/StatePassive.cpp
    src/FSM/StateFixedDown.cpp
    src/FSM/StateFreeStand.cpp
    src/FSM/StateMPCTrotting.cpp
    src/FSM/StateFixedStand.cpp
    src/robot/QuadrupedRobot.cpp
    src/control/LowPassFilter.cpp
    src/control/Estimator.cpp
    src/control/LegController.cpp
    src/gait/Gait.cpp
    src/robot/go2_robot_data/PinGo2Model.cpp
    src/mpc/ComTrajectory.cpp
    src/mpc/CentroidalMPC.cpp
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
  OsqpEigen::OsqpEigen
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH "${QUADRUPED_THIRD_PARTY_RPATH}"
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
