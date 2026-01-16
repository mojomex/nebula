cmake_minimum_required(VERSION 3.20)
project(nebula)
set(ignoreMe "${CMAKE_EXPORT_COMPILE_COMMANDS}")

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
