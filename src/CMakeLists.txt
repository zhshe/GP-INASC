# find .cpp files 
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/ground_extraction GROUND_SRCS)

###### build ########
add_executable(${PROJECT_NAME}_node main.cpp ${GROUND_SRCS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

###### test ########
add_subdirectory(test)