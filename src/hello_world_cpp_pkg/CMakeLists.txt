cmake_minimum_required(VERSION 3.8)
project(hello_world_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hello_world_node src/hello_world_node.cpp)
ament_target_dependencies(hello_world_node rclcpp)

add_executable(custom_node src/custom_node.cpp)
ament_target_dependencies(custom_node rclcpp)

add_executable(simple_pub src/simple_pub.cpp)
ament_target_dependencies(simple_pub rclcpp std_msgs)

add_executable(simple_sub src/simple_sub.cpp)
ament_target_dependencies(simple_sub rclcpp std_msgs)

install(
    TARGETS 
    hello_world_node
    custom_node
    simple_pub
    simple_sub
    # add more executables here if needed
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

ament_package()
