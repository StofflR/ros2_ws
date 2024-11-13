cmake_minimum_required(VERSION 3.8)
project(husky_highlevel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(subscriber "src/subscriber.cpp")
add_executable(distance_stop "src/distance_stop.cpp")
add_executable(crash_stop "src/crash_stop.cpp")

target_compile_features(subscriber PUBLIC cxx_std_17)
target_compile_features(distance_stop PUBLIC cxx_std_17)
target_compile_features(crash_stop PUBLIC cxx_std_17)
ament_target_dependencies(subscriber rclcpp sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs)
ament_target_dependencies(distance_stop rclcpp sensor_msgs std_srvs)
ament_target_dependencies(crash_stop rclcpp sensor_msgs std_srvs)

target_link_libraries(subscriber ${cpp_typesupport_target})
target_link_libraries(distance_stop ${cpp_typesupport_target})
target_link_libraries(crash_stop ${cpp_typesupport_target})

install(
  TARGETS
    subscriber
    distance_stop
    crash_stop
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
