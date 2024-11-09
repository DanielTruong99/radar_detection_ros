cmake_minimum_required(VERSION 3.8)
project(collision_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/ryz2/DanielWorkspace/libtorch)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Torch REQUIRED)

# add dependencies
add_subdirectory(src/CollisionDetector)

# add executables
add_executable(demo_torch_node src/demo_node.cpp)
target_link_libraries(demo_torch_node CollisionDetector)
ament_target_dependencies(demo_torch_node rclcpp std_msgs)

add_executable(demo_torch_node_02 src/demo_node_02.cpp)
target_link_libraries(demo_torch_node_02 CollisionDetector)
ament_target_dependencies(demo_torch_node_02 rclcpp std_msgs)

# install executables
install(TARGETS demo_torch_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS demo_torch_node_02 DESTINATION lib/${PROJECT_NAME})

# install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
