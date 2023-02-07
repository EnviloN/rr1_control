cmake_minimum_required(VERSION 3.8)
project(rr1_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)


# Add a c++ executables
# add_executable(cpp_executable src/cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)

install(TARGETS
  # cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install the python module for this package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/angles_converter.py
  scripts/controller_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
