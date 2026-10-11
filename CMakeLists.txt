cmake_minimum_required(VERSION 3.20)
project(serial_can_bridge LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED CONFIG)
find_package(Threads REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_library(serial_can_bridge_core SHARED
  src/protocol.cpp
  src/serial_bridge.cpp
  src/serial_can_bridge_node.cpp)
target_compile_features(serial_can_bridge_core PUBLIC cxx_std_17)
target_include_directories(serial_can_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(serial_can_bridge_core PUBLIC
  Boost::headers Threads::Threads ${can_msgs_TARGETS}
  rclcpp::rclcpp ${rcl_interfaces_TARGETS})

add_executable(serial_can_bridge src/main.cpp)
target_link_libraries(serial_can_bridge PRIVATE serial_can_bridge_core)

install(TARGETS serial_can_bridge_core EXPORT serial_can_bridge_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS serial_can_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(serial_can_bridge_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Threads can_msgs rclcpp rcl_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name IN ITEMS serial_bridge protocol node)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp TIMEOUT 15)
    target_link_libraries(test_${test_name} serial_can_bridge_core)
  endforeach()
  target_link_libraries(test_serial_bridge util)
  target_link_libraries(test_node util)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
