add_library(teleop_joy
            teleop_joy.cpp teleop_joy.h)
target_link_libraries(teleop_joy
                      ${catkin_LIBRARIES})
                      
add_executable(teleop_joy_main.exe
               teleop_joy_main.cpp)
target_link_libraries(teleop_joy_main.exe
                      teleop_joy ${catkin_LIBRARIES})
