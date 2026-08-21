cmake_minimum_required(VERSION 3.20)

project(message_filters)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
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

add_library(${PROJECT_NAME}
  src/connection.cpp
  src/message_event.cpp)
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
  std_msgs::std_msgs
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
  set(ament_cmake_cppcheck_LANGUAGE "c++")
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(sensor_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)

  ament_add_gtest(${PROJECT_NAME}-test_policies
    test/test_main.cpp
    test/test_exact_time_policy.cpp
    test/test_approximate_time_policy.cpp
    test/test_approximate_epsilon_time_policy.cpp
    test/test_latest_time_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_policies)
    target_link_libraries(${PROJECT_NAME}-test_policies ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_sync
    test/test_main.cpp
    test/test_synchronizer.cpp
    test/time_synchronizer_unittest.cpp
    test/test_input_aligner.cpp)
  if(TARGET ${PROJECT_NAME}-test_sync)
    target_link_libraries(${PROJECT_NAME}-test_sync ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_filters
    test/test_main.cpp
    test/test_simple.cpp
    test/test_chain.cpp
    test/msg_cache_unittest.cpp
    test/time_sequencer_unittest.cpp
    test/test_message_traits.cpp
    test/test_message_filters_delta_filter.cpp
    test/test_custom_time_getter.cpp)
  if(TARGET ${PROJECT_NAME}-test_filters)
    target_link_libraries(${PROJECT_NAME}-test_filters ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_subscriber test/test_subscriber.cpp)
  if(TARGET ${PROJECT_NAME}-test_subscriber)
    target_link_libraries(${PROJECT_NAME}-test_subscriber ${PROJECT_NAME} rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle sensor_msgs::sensor_msgs)
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_fuzz test/test_fuzz.cpp SKIP_TEST)
  if(TARGET ${PROJECT_NAME}-test_fuzz)
    target_link_libraries(${PROJECT_NAME}-test_fuzz ${PROJECT_NAME} rclcpp::rclcpp sensor_msgs::sensor_msgs)
  endif()

  # python tests with python interfaces of message filters
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_time_synchronizer.py "test/test_time_synchronizer.py")
  ament_add_pytest_test(test_approxsync.py "test/test_approxsync.py")
  ament_add_pytest_test(test_message_filters_cache.py "test/test_message_filters_cache.py")
  ament_add_pytest_test(test_message_filters_chain.py "test/test_message_filters_chain.py")
  ament_add_pytest_test(test_input_aligner.py "test/test_input_aligner.py")
  ament_add_pytest_test(test_custom_time_getter.py "test/test_custom_time_getter.py")
endif()

ament_package()
