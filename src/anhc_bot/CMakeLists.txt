cmake_minimum_required(VERSION 3.5)
project(anhc_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes models urdf worlds rviz config scripts docs
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
