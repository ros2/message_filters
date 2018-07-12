cmake_minimum_required(VERSION 3.5)

project(message_filters)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)
add_library(${PROJECT_NAME} SHARED src/connection.cpp)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rcutils"
  "std_msgs"
  "builtin_interfaces"
  "sensor_msgs")

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
DIRECTORY "include/"
DESTINATION include
)

find_package(python_cmake_module REQUIRED)
_ament_cmake_python_register_environment_hook()
install(
  DIRECTORY "src/${PROJECT_NAME}/"
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
