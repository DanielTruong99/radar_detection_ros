add_library(CollisionDetector STATIC CollisionDetector.h CollisionDetector.cpp)
target_include_directories(CollisionDetector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${TORCH_INCLUDE_DIRS})

target_link_libraries(CollisionDetector ${TORCH_LIBRARIES})
ament_target_dependencies(CollisionDetector rclcpp)