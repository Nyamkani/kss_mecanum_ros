cmake_minimum_required(VERSION 3.8)
project(arduino_motor_driver_bridge)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



#ROS package
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)


# make excutable
add_executable(arduino_motor_driver_bridge src/main.cpp ${sources})
#ROS package
ament_target_dependencies(arduino_motor_driver_bridge 
                          rclcpp 
                          std_msgs 
                          geometry_msgs 
                          nav_msgs
                          tf2_ros
                          tf2
                          sensor_msgs
                          )

# Get register all includes
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )

target_link_libraries(${PROJECT_NAME}
  # isv2motor
  # pgv100
  )

# make library
# add_library(${PROJECT_NAME}
#   ${sources}
# )

install(TARGETS
  arduino_motor_driver_bridge
  DESTINATION lib/${PROJECT_NAME}  
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#ROS package
ament_package()
