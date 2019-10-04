cmake_minimum_required(VERSION 3.5)
project(franka_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(Franka 0.7.1 REQUIRED)

include_directories(include)

set(APPS
  robot_state_publisher
  tactile_sensor_driver
)

foreach(target ${APPS})
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} pcanbasic Franka::Franka)
  ament_target_dependencies(${target}
    "rclcpp"
    "franka_msgs")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()