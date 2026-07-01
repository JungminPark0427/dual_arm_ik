cmake_minimum_required(VERSION 3.0.2)
project(dual_arm_ik)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(pinocchio REQUIRED)

add_definitions(-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_LIST_SIZE=30)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dual_arm_ik_node src/dual_arm_ik_node.cpp)
add_dependencies(dual_arm_ik_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_arm_ik_node ${catkin_LIBRARIES} pinocchio::pinocchio)
set_target_properties(dual_arm_ik_node PROPERTIES CXX_STANDARD 17)

add_executable(fk_display_node src/fk_display_node.cpp)
add_dependencies(fk_display_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fk_display_node ${catkin_LIBRARIES} pinocchio::pinocchio)
set_target_properties(fk_display_node PROPERTIES CXX_STANDARD 17)