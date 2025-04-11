cmake_minimum_required(VERSION 3.8)
project(ebot_nav2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Order.msg"
  "msg/State.msg"
  "srv/Delivery.srv"
  "srv/Confirmation.srv"
  "srv/OrderOrCancel.srv"
  "srv/StateUpdater.srv"
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

install(
  DIRECTORY
    launch
    config
    maps
    params
    rviz
    scripts
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/set_initial_pose.py
  scripts/order_delivery_service.py
  scripts/confirmation_service.py
  scripts/order_manager.py
  scripts/order_or_cancel_service.py
  scripts/state_updater_service.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
