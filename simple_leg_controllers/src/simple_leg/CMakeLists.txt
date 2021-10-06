# simple_leg_hw
add_executable(simple_leg_control   simple_leg_control.cpp simple_leg_hw.cpp)
add_dependencies(simple_leg_control   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_leg_control  ${catkin_LIBRARIES})

# simple_leg_hw_sim
add_library(simple_leg_hw_sim simple_leg_hw_sim.cpp)
target_link_libraries(simple_leg_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
install (FILES simple_leg_hw_sim_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# leg_position_publisher
add_executable(leg_position_publisher   leg_position_publisher.cpp simple_leg_hw.cpp)
add_dependencies(leg_position_publisher   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(leg_position_publisher  ${catkin_LIBRARIES})