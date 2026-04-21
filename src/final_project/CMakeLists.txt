cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(abb_irb4600_ikfast REQUIRED)
find_package(std_msgs REQUIRED)

add_library(manipulator_lib src/manipulator.cpp)
target_include_directories(manipulator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(manipulator_lib
  abb_irb4600_ikfast
)

add_executable(model_spawner src/model_spawner.cpp)
ament_target_dependencies(model_spawner
  rclcpp
  visualization_msgs
)

add_executable(pose_teacher src/pose_teacher.cpp)
ament_target_dependencies(pose_teacher
  rclcpp
  visualization_msgs
  sensor_msgs
  geometry_msgs
  interactive_markers
  abb_irb4600_ikfast
)
target_include_directories(pose_teacher PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(ptp_via_demo src/ptp_via_demo.cpp)
ament_target_dependencies(ptp_via_demo
  rclcpp
  sensor_msgs
)
target_link_libraries(ptp_via_demo manipulator_lib)

add_executable(lin_demo src/lin_demo.cpp)
ament_target_dependencies(lin_demo
  rclcpp
  sensor_msgs
  abb_irb4600_ikfast
)
target_link_libraries(lin_demo manipulator_lib)
target_include_directories(lin_demo PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(approach_demo src/approach_demo.cpp)
ament_target_dependencies(approach_demo
  rclcpp
  sensor_msgs
  abb_irb4600_ikfast
)
target_link_libraries(approach_demo manipulator_lib)
target_include_directories(approach_demo PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(fk_dump src/fk_dump.cpp)
ament_target_dependencies(fk_dump
  rclcpp
  abb_irb4600_ikfast
)
target_include_directories(fk_dump PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(manipulator_executor src/manipulator_executor.cpp)
ament_target_dependencies(manipulator_executor
  rclcpp
  sensor_msgs
  std_msgs
  abb_irb4600_ikfast
)
target_link_libraries(manipulator_executor manipulator_lib)
target_include_directories(manipulator_executor PRIVATE ${EIGEN3_INCLUDE_DIRS})

install(TARGETS
  model_spawner
  pose_teacher
  ptp_via_demo
  manipulator_lib
  lin_demo
  approach_demo
  fk_dump
  manipulator_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()