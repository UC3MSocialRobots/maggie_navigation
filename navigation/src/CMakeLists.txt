add_executable(laser_max_range_filter.exe
               laser_max_range_filter.cpp)
target_link_libraries(laser_max_range_filter.exe
                      ${catkin_LIBRARIES})

add_executable(teleop_joy.exe
               teleop_joy.cpp)
target_link_libraries(teleop_joy.exe
                      ${catkin_LIBRARIES})

add_executable(teleop_keyboard
               teleop_keyboard.cpp)
target_link_libraries(teleop_keyboard
                      ${catkin_LIBRARIES})

add_executable(initial_status_sender
               initial_status_sender.cpp)
target_link_libraries(initial_status_sender
                      ${catkin_LIBRARIES})

#add_executable(state_estimator
               #state_estimator.cpp state.cpp)
#target_link_libraries(state_estimator
                      #SDL SDL_image ${catkin_LIBRARIES})
#
#add_executable(mdp_controller
               #mdp_controller.cpp state.cpp)
#target_link_libraries(mdp_controller
                      #${catkin_LIBRARIES})
#
#add_executable(patrol_area_server
               #patrol_area_server.cpp)
#target_link_libraries(patrol_area_server
                      #${catkin_LIBRARIES})
#
#add_executable(move_to_event_position_server
               #move_to_event_position_server.cpp)
#target_link_libraries(move_to_event_position_server
                      #${catkin_LIBRARIES})
#
#add_executable(maiss_stuck_robot_monitor
               #stuck_robot_monitor.cpp)
#target_link_libraries(maiss_stuck_robot_monitor
                      #${catkin_LIBRARIES})
