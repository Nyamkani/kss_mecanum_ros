cmake_minimum_required(VERSION 3.8)
project(mecanum_gateway)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ZLIB REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)

add_executable(gateway_node
  src/main.cpp
  src/gateway_node.cpp
  src/navigation_client.cpp
  src/tcp_server.cpp
  src/map_stream.cpp
  src/mode_manager.cpp
  src/protocol.cpp
)
target_compile_features(gateway_node PRIVATE cxx_std_17)
set_target_properties(gateway_node PROPERTIES CXX_EXTENSIONS OFF)
target_compile_options(gateway_node PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(gateway_node PRIVATE include)
ament_target_dependencies(gateway_node rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs tf2 tf2_ros)
target_link_libraries(gateway_node nlohmann_json::nlohmann_json Threads::Threads ZLIB::ZLIB)

install(TARGETS gateway_node DESTINATION lib/${PROJECT_NAME})
include(CTest)
if(BUILD_TESTING)
  add_executable(mode_manager_test test/mode_manager_test.cpp src/mode_manager.cpp src/protocol.cpp)
  target_compile_features(mode_manager_test PRIVATE cxx_std_17)
  target_include_directories(mode_manager_test PRIVATE include)
  target_link_libraries(mode_manager_test nlohmann_json::nlohmann_json Threads::Threads)
  add_test(NAME mode_manager_test COMMAND mode_manager_test)
  set_tests_properties(mode_manager_test PROPERTIES TIMEOUT 25)
  add_executable(map_stream_test test/map_stream_test.cpp src/map_stream.cpp src/protocol.cpp)
  target_compile_features(map_stream_test PRIVATE cxx_std_17)
  target_include_directories(map_stream_test PRIVATE include)
  ament_target_dependencies(map_stream_test nav_msgs)
  target_link_libraries(map_stream_test nlohmann_json::nlohmann_json Threads::Threads ZLIB::ZLIB)
  add_test(NAME map_stream_test COMMAND map_stream_test)
  set_tests_properties(map_stream_test PROPERTIES TIMEOUT 10)
endif()
ament_package()
