cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY dae meshes xacro launch config urdf mujoco
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()
