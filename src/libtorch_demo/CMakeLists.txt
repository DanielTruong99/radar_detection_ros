cmake_minimum_required(VERSION 3.0.2)
project(libtorch_demo)

message("CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /home/ryz2/DanielWorkspace/libtorch)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Torch REQUIRED)

catkin_package()

add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)
add_subdirectory(src/ColisionDetector)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

add_executable(demo_torch_node src/demo_node.cpp)

target_link_libraries(demo_torch_node PUBLIC ${catkin_LIBRARIES} ColisionDetector)
