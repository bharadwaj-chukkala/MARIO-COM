cmake_minimum_required(VERSION 3.5)
project(mario_com)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
  set(ARG_CONFIG_FILE "${PROJECT_SOURCE_DIR}/uncrustify_google_cpp_allman.cfg"
    CACHE STRING "uncrustify config file")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_lint_auto_find_test_dependencies()

  set(TESTFILES
    test/main.cpp
    test/manipulation_test.cpp
    test/navigation_test.cpp
    test/perception_test.cpp
    src/Manipulation.cpp
    src/Navigation.cpp
    src/Perception.cpp
    src/RobotSim.cpp
  )
  # Add gtests
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs)
  target_include_directories(test_${PROJECT_NAME} PUBLIC ../include/mario_com)
  # rosidl_target_interfaces(server_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  
  install(TARGETS
    test_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}  
  )
endif()

# include_directories(
#     ${CMAKE_SOURCE_DIR}/include/mario_com
# )
# add_executable(manipulation )
# ament_target_dependencies(manipulation rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs OpenCV)

# add_executable(navigation )
# ament_target_dependencies(navigation rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs OpenCV)

# add_executable(perception )
# ament_target_dependencies(perception rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs OpenCV)

# add_executable(robotsim )
# ament_target_dependencies(robotsim rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs OpenCV)

add_executable(my_main src/main.cpp src/Manipulation.cpp src/Navigation.cpp src/Perception.cpp src/RobotSim.cpp)
ament_target_dependencies(my_main rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs)
include_directories(
    ${CMAKE_SOURCE_DIR}/include/mario_com
)

# add_executable(test_odom src/sub_test.cpp)
# ament_target_dependencies(test_odom rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs OpenCV nav_msgs)
# include_directories(
#     ${CMAKE_SOURCE_DIR}/include/mario_com
# ) 

# add_executable(test_img src/sub_image.cpp)
# ament_target_dependencies(test_img rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs)
# include_directories(
#     ${CMAKE_SOURCE_DIR}/include/mario_com
# )


install(TARGETS
  my_main
  # test_odom
  # test_img
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  worlds
  models
  maps
  DESTINATION share/${PROJECT_NAME}
)

## Install some arbitrary executables
install(PROGRAMS      # want to make the file executable (ie. chmod a+x)
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})

ament_package()
