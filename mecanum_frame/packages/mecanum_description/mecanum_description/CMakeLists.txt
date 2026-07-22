cmake_minimum_required(VERSION 3.8)
project(mecanum_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

file(GLOB_RECURSE sources CONFIGURE_DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

# make excutable
add_executable(mecanum_description src/main.cpp ${sources})
#ROS package
ament_target_dependencies(mecanum_description 
                          rclcpp 
                          urdf 
                          xacro 

                          )

# Get register all includes
target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  )

install(TARGETS
mecanum_description
  DESTINATION share/${PROJECT_NAME}  
)

install(
  DIRECTORY urdf meshes launch config
  DESTINATION share/${PROJECT_NAME}  
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only woa git repo)
  # comment the line when this package is in a git rks in repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")


ament_package()
