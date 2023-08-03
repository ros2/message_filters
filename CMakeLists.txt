cmake_minimum_required(VERSION 3.5)

project(message_filters)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} src/connection.cpp)
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "MESSAGE_FILTERS_BUILDING_DLL")
endif()
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rcutils::rcutils
  ${std_msgs_TARGETS}
)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "include/"
  DESTINATION include/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(rclcpp rcutils std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(sensor_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  ament_add_gtest(${PROJECT_NAME}-test_simple test/test_simple.cpp)
  if(TARGET ${PROJECT_NAME}-test_simple)
    target_link_libraries(${PROJECT_NAME}-test_simple ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-msg_cache_unittest test/msg_cache_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-msg_cache_unittest)
    target_link_libraries(${PROJECT_NAME}-msg_cache_unittest ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_chain test/test_chain.cpp)
  if(TARGET ${PROJECT_NAME}-test_chain)
    target_link_libraries(${PROJECT_NAME}-test_chain ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_time_sequencer test/time_sequencer_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-test_time_sequencer)
    target_link_libraries(${PROJECT_NAME}-test_time_sequencer ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_subscriber test/test_subscriber.cpp)
  if(TARGET ${PROJECT_NAME}-test_subscriber)
    target_link_libraries(${PROJECT_NAME}-test_subscriber ${PROJECT_NAME} rclcpp::rclcpp rclcpp_lifecycle::rclcpp_lifecycle ${sensor_msgs_TARGETS})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_synchronizer test/test_synchronizer.cpp)
  if(TARGET ${PROJECT_NAME}-test_synchronizer)
    target_link_libraries(${PROJECT_NAME}-test_synchronizer ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-time_synchronizer_unittest test/time_synchronizer_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-time_synchronizer_unittest)
    target_link_libraries(${PROJECT_NAME}-time_synchronizer_unittest ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_exact_time_policy test/test_exact_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_exact_time_policy)
    target_link_libraries(${PROJECT_NAME}-test_exact_time_policy ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_approximate_time_policy test/test_approximate_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_approximate_time_policy)
    target_link_libraries(${PROJECT_NAME}-test_approximate_time_policy ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_approximate_epsilon_time_policy test/test_approximate_epsilon_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_approximate_epsilon_time_policy)
    target_link_libraries(${PROJECT_NAME}-test_approximate_epsilon_time_policy ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_latest_time_policy test/test_latest_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_latest_time_policy)
    target_link_libraries(${PROJECT_NAME}-test_latest_time_policy ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_fuzz test/test_fuzz.cpp SKIP_TEST)
  if(TARGET ${PROJECT_NAME}-test_fuzz)
    target_link_libraries(${PROJECT_NAME}-test_fuzz ${PROJECT_NAME} rclcpp::rclcpp ${sensor_msgs_TARGETS})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_message_traits test/test_message_traits.cpp)
  if(TARGET ${PROJECT_NAME}-test_message_traits)
    target_link_libraries(${PROJECT_NAME}-test_message_traits ${PROJECT_NAME} rclcpp::rclcpp ${std_msgs_TARGETS})
  endif()

  # Provides PYTHON_EXECUTABLE_DEBUG
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)
  set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  if(WIN32)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
      set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
    endif()
  endif()

  # python tests with python interfaces of message filters
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(directed.py "test/directed.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")
  ament_add_pytest_test(test_approxsync.py "test/test_approxsync.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")
  ament_add_pytest_test(test_message_filters_cache.py "test/test_message_filters_cache.py"
    PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")
endif()

ament_package()
