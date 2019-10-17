cmake_minimum_required(VERSION 3.5)
project(franka_control)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.1 REQUIRED)
find_package(Threads REQUIRED)

include_directories(include)

add_library(control_common STATIC
  src/control_common.cc)
target_link_libraries(control_common PUBLIC
  Eigen3::Eigen3
  Franka::Franka)

set(APPS
  control_interface
  tactile_sensor_driver
)

foreach(target ${APPS})
  add_executable(${target} src/${target}.cc)
  target_link_libraries(${target}     
    control_common 
    pcanbasic
    Franka::Franka
    Threads::Threads)
  ament_target_dependencies(${target}
    "franka_msgs"
    "rclcpp")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
