cmake_minimum_required(VERSION 3.0.2)
project(carry_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf
  upros_message
)

catkin_package(
  CATKIN_DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(print_id src/print_id.cpp)
target_link_libraries(print_id ${catkin_LIBRARIES})

add_executable(arm_put src/arm_put.cpp)
target_link_libraries(arm_put ${catkin_LIBRARIES})

add_executable(arm_reset src/arm_reset.cpp)
target_link_libraries(arm_reset ${catkin_LIBRARIES})

add_executable(w4a_movebase_client src/movebase_client.cpp)
target_link_libraries(w4a_movebase_client ${catkin_LIBRARIES})

add_executable(w4a_tf_echo_node src/tf_echo.cpp)
target_link_libraries(w4a_tf_echo_node ${catkin_LIBRARIES})

add_executable(w4a_arm_grab_node src/arm_grab.cpp)
target_link_libraries(w4a_arm_grab_node ${catkin_LIBRARIES})

add_executable(print_id_test src/print_id_test.cpp)
target_link_libraries(print_id_test ${catkin_LIBRARIES})

add_executable(w4a_task_flow_node src/task_flow.cpp)
target_link_libraries(w4a_task_flow_node ${catkin_LIBRARIES})
add_executable(complete_flow_mm_m src/complete_flow_mm_m.cpp)
target_link_libraries(complete_flow_mm_m ${catkin_LIBRARIES})
