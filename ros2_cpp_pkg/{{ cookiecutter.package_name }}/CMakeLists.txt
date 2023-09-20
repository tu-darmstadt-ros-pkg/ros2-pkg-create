cmake_minimum_required(VERSION 3.8)
project({{ cookiecutter.package_name }})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable({{ cookiecutter.node_name }} src/{{ cookiecutter.node_name }}.cpp)

target_include_directories({{ cookiecutter.node_name }} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features({{ cookiecutter.node_name }} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies({{ cookiecutter.node_name }}
  rclcpp
)

install(TARGETS {{ cookiecutter.node_name }}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
