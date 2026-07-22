cmake_minimum_required(VERSION 3.8)
project(kssekf)

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
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(message_filters REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)


# make excutable
add_executable(kssekf src/main.cpp ${sources})
#ROS package
ament_target_dependencies(kssekf 
                          rclcpp 
                          std_msgs 
                          geometry_msgs 
                          nav_msgs
                          tf2_ros
                          tf2
                          sensor_msgs
                          message_filters
                          Eigen3
                          )

# Get register all includes
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${EIGEN3_INCLUDE_DIR}
  /usr/local/include
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
  kssekf
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

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
