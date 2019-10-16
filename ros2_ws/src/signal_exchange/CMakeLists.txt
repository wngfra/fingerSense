cmake_minimum_required(VERSION 3.5)
project(signal_exchange)

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
find_package(Franka 0.7.1 REQUIRED)

# include_directories(include)

set(APPS
  robot_state_publisher
  tactile_sensor_driver
)

foreach(target ${APPS})
  add_executable(${target} src/${target}.cc)
  target_link_libraries(${target} pcanbasic Franka::Franka)
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
  DESTINATION share/${PROJECT_NAME}  
)

ament_package()