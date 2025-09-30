cmake_minimum_required(VERSION 3.8)
project(gripper_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# Look for the gpiod library
find_library(GPIOD_LIB gpiod)
find_path(GPIOD_INCLUDE_DIR gpiod.h)

if(NOT GPIOD_LIB OR NOT GPIOD_INCLUDE_DIR)
  message(FATAL_ERROR "libgpiod-dev not found. Please install with: sudo apt install libgpiod-dev")
endif()

# install node
add_executable(gripper_controller src/main.cpp src/gripper_controller.cpp)
ament_target_dependencies(gripper_controller rclcpp std_srvs std_msgs)
target_include_directories(gripper_controller PRIVATE include ${GPIOD_INCLUDE_DIR})
target_link_libraries(gripper_controller ${GPIOD_LIB})

install(TARGETS gripper_controller
  DESTINATION lib/${PROJECT_NAME})

# install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
