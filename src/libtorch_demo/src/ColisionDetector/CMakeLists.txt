add_library(ColisionDetector STATIC ColisionDetector.h ColisionDetector.cpp)
target_include_directories(ColisionDetector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${catkin_INCLUDE_DIRS} ${TORCH_INCLUDE_DIRS})

target_link_libraries(ColisionDetector ${catkin_LIBRARIES} ${TORCH_LIBRARIES})
