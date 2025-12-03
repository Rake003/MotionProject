cmake_minimum_required(VERSION 3.8)
project(flexiv_kinematics_ros)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kinematic_and_dynamic_utils REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# --------------------------------------
# Build library: flexiv_ros_kinematics_utils
# --------------------------------------
add_library(flexiv_ros_kinematics_utils SHARED src/flexiv_ros_kinematics_utils.cpp)

target_include_directories(flexiv_ros_kinematics_utils PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(flexiv_ros_kinematics_utils PUBLIC
  kinematic_and_dynamic_utils::kdl_robot_model
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

target_include_directories(flexiv_ros_kinematics_utils PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# --------------------------------------
# Build example node
# --------------------------------------
add_executable(example_node src/example_node.cpp)
target_include_directories(example_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(example_node PUBLIC
                                        flexiv_ros_kinematics_utils
                                        kinematic_and_dynamic_utils::kdl_robot_model
                                        ${rclcpp_LIBRARIES}
                                        ${sensor_msgs_LIBRARIES}
                                        ${geometry_msgs_LIBRARIES})
target_include_directories(example_node PUBLIC
                                              ${rclcpp_INCLUDE_DIRS}
                                              ${sensor_msgs_INCLUDE_DIRS}
                                              ${geometry_msgs_INCLUDE_DIRS})

# --------------------------------------
# Build nullspace_control
# --------------------------------------
add_executable(nullspace_control src/nullspace_control.cpp)
target_include_directories(nullspace_control PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(nullspace_control PUBLIC
                                        flexiv_ros_kinematics_utils
                                        kinematic_and_dynamic_utils::kdl_robot_model
                                        ${rclcpp_LIBRARIES}
                                        ${sensor_msgs_LIBRARIES}
                                        ${geometry_msgs_LIBRARIES})
target_include_directories(nullspace_control PUBLIC
                                              ${rclcpp_INCLUDE_DIRS}
                                              ${sensor_msgs_INCLUDE_DIRS}
                                              ${geometry_msgs_INCLUDE_DIRS})

# --------------------------------------
# Build null_space_example
# --------------------------------------
add_executable(null_space_example src/null_space_example.cpp)
target_include_directories(null_space_example PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(null_space_example PUBLIC
                                        flexiv_ros_kinematics_utils
                                        kinematic_and_dynamic_utils::kdl_robot_model
                                        ${rclcpp_LIBRARIES}
                                        ${sensor_msgs_LIBRARIES}
                                        ${geometry_msgs_LIBRARIES})
target_include_directories(null_space_example PUBLIC
                                              ${rclcpp_INCLUDE_DIRS}
                                              ${sensor_msgs_INCLUDE_DIRS}
                                              ${geometry_msgs_INCLUDE_DIRS})                                              

# --------------------------------------
# Install targets
# --------------------------------------
install(TARGETS
              flexiv_ros_kinematics_utils
              example_node
              nullspace_control
              null_space_example
              EXPORT ${PROJECT_NAME}Targets
              LIBRARY DESTINATION lib
              ARCHIVE DESTINATION lib
              RUNTIME DESTINATION lib/${PROJECT_NAME}
              INCLUDES DESTINATION include)

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Targets.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME})

# --------------------------------------
# Install include files (headers)
# --------------------------------------
install(DIRECTORY include/ DESTINATION include)

# --------------------------------------
# Install URDF, launch, rviz assets
# --------------------------------------
install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)

# Export dependencies
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp
                          sensor_msgs
                          geometry_msgs
                          Eigen3
                          kinematic_and_dynamic_utils)
# Export package
ament_package()