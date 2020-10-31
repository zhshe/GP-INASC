add_executable(${PROJECT_NAME}_node_test test.cpp ${GROUND_SRCS})
target_link_libraries(${PROJECT_NAME}_node_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})