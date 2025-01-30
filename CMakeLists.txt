cmake_minimum_required(VERSION 3.5)
project(ros2_pkg_create)

find_package(ament_cmake REQUIRED)


ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY templates DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)
install(FILES copier.yml DESTINATION share/${PROJECT_NAME})

ament_package()