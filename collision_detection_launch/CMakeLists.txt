cmake_minimum_required(VERSION 3.5)
project(collision_detection_launch)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()


find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_ros_perception REQUIRED)
# find_package(moveit_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(collision_detection_msgs REQUIRED)
#finde moveit packages

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
  collision_detection_msgs
  std_srvs
)
include_directories(include)

# add_library(${PROJECT_NAME} SHARED
#   src/trajectory_generator_node.cpp
# )
add_executable(trajectory_generator_node
  src/trajectory_generator_node.cpp
  src/trajectory_generator.cpp
)
# target_include_directories(${PROJECT_NAME} PRIVATE
#   include
# )

ament_target_dependencies(trajectory_generator_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)


install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
