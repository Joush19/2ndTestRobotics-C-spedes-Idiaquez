cmake_minimum_required(VERSION 3.8)
project(test_move)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(collition src/move_constrain.cpp)
target_include_directories(collition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(collition PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  collition
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(draw_a src/a_path.cpp)
target_include_directories(draw_a PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(draw_a PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  draw_a
  "moveit_ros_planning_interface"
  "rclcpp"
)








add_executable(ej2 src/ej2.cpp)
target_include_directories(ej2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ej2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ej2
  "moveit_ros_planning_interface"
  "rclcpp"
)








add_executable(ejercicio2 src/ejercicio2.cpp)
target_include_directories(ejercicio2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ejercicio2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ejercicio2
  "moveit_ros_planning_interface"
  "rclcpp"
)



add_executable(obstaculo src/obstaculo.cpp)
target_include_directories(obstaculo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(obstaculo PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  obstaculo
  "moveit_ros_planning_interface"
  "rclcpp"
)








install(TARGETS 
  hello_moveit 
  collition 
  draw_a
  ej2
  ejercicio2
  obstaculo
  DESTINATION lib/${PROJECT_NAME})

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
