cmake_minimum_required(VERSION 3.8)
project(hardcode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

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


add_executable(basic_spawner src/span_turtle_basic.cpp)
ament_target_dependencies(basic_spawner rclcpp turtlesim)


add_executable(loop_spawner src/loop_spawn.cpp)
ament_target_dependencies(loop_spawner rclcpp turtlesim)


add_executable(class_spawner src/Class_Turtle.cpp)
ament_target_dependencies(class_spawner rclcpp turtlesim geometry_msgs) 



install(TARGETS
  basic_spawner
  loop_spawner
  class_spawner
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
