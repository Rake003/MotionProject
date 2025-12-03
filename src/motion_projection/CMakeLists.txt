cmake_minimum_required(VERSION 3.8)
project(motion_projection)

# --------------------------------------
# Compiler settings
# --------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --------------------------------------
# Dependencies
# --------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kinematic_and_dynamic_utils REQUIRED)
find_package(scancraft REQUIRED)

# ======================================================
# Vendored Open3D only (do not probe system paths)
# ======================================================
# Use the Open3D copy shipped inside the workspace at `Open3D/`.
set(Open3D_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Open3D)
set(Open3D_INCLUDE_DIRS ${Open3D_ROOT}/include)
set(Open3D_MAIN_LIB ${Open3D_ROOT}/lib/libOpen3D.so)

if(EXISTS ${Open3D_INCLUDE_DIRS} AND EXISTS ${Open3D_MAIN_LIB})
    set(OPEN3D_FOUND TRUE)
    message(STATUS "Using vendored Open3D from ${Open3D_ROOT}")
else()
    message(FATAL_ERROR "Vendored Open3D not found in ${Open3D_ROOT}. Place Open3D headers in ${Open3D_ROOT}/include and the library in ${Open3D_ROOT}/lib/libOpen3D.so.")
endif()

# --------------------------------------
# Include dirs
# --------------------------------------
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)

# ============================================================
# Library: ros_kinematics_utility
# ============================================================
add_library(ros_kinematics_utility SHARED src/ros_kinematics_utility.cpp)
target_include_directories(ros_kinematics_utility PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ros_kinematics_utility PUBLIC
  Eigen3::Eigen
  kinematic_and_dynamic_utils::kdl_robot_model
)
ament_target_dependencies(ros_kinematics_utility PUBLIC
  rclcpp rcl std_msgs sensor_msgs geometry_msgs visualization_msgs
)

# ============================================================
# Library: helper_functions  (uses Open3D)
# ============================================================
add_library(helper_functions SHARED src/helper_functions.cpp)
target_include_directories(helper_functions PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(OPEN3D_FOUND)
  target_include_directories(helper_functions PUBLIC 
    $<BUILD_INTERFACE:${Open3D_INCLUDE_DIRS}> 
    $<INSTALL_INTERFACE:include/Open3D>
  )
endif()
target_link_libraries(helper_functions PUBLIC
  Eigen3::Eigen
)
if(OPEN3D_FOUND)
  target_link_libraries(helper_functions PUBLIC ${Open3D_MAIN_LIB} c++ c++abi)
endif()
ament_target_dependencies(helper_functions PUBLIC
  rclcpp rcl std_msgs sensor_msgs geometry_msgs
)

# ============================================================
# Library: mesh_projection
# ============================================================
add_library(mesh_projection SHARED src/mesh_projection.cpp)
target_include_directories(mesh_projection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mesh_projection PUBLIC
  Eigen3::Eigen
  scancraft::recon_lib
)
ament_target_dependencies(mesh_projection PUBLIC
  rclcpp rcl std_msgs sensor_msgs geometry_msgs
)

# Set RPATH for libraries that use Open3D so they can find it at runtime
if(OPEN3D_FOUND)
  get_filename_component(OPEN3D_LIB_DIR ${Open3D_MAIN_LIB} DIRECTORY)
  set_target_properties(helper_functions PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_RPATH "${OPEN3D_LIB_DIR}"
  )
endif()

# ============================================================
# Library: mesh_visualization
# ============================================================
add_library(mesh_visualization SHARED src/mesh_visualization.cpp)
target_include_directories(mesh_visualization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mesh_visualization PUBLIC
  Eigen3::Eigen
  scancraft::recon_lib
)
ament_target_dependencies(mesh_visualization PUBLIC
  rclcpp rcl std_msgs sensor_msgs geometry_msgs visualization_msgs
)

# Set RPATH for libraries that use Open3D so they can find it at runtime
if(OPEN3D_FOUND)
  get_filename_component(OPEN3D_LIB_DIR ${Open3D_MAIN_LIB} DIRECTORY)
  set_target_properties(helper_functions PROPERTIES
    INSTALL_RPATH "$ORIGIN"
    BUILD_RPATH "${OPEN3D_LIB_DIR}"
  )
endif()

# ============================================================
# Executables
# ============================================================

add_executable(example_node src/example_node.cpp)
target_link_libraries(example_node PUBLIC ros_kinematics_utility)

add_executable(example_generate_mesh src/example_generate_mesh.cpp)
target_link_libraries(example_generate_mesh PUBLIC helper_functions)
if(OPEN3D_FOUND)
  target_link_libraries(example_generate_mesh PUBLIC ${Open3D_MAIN_LIB} c++ c++abi)
  set_target_properties(example_generate_mesh PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_RPATH "${OPEN3D_LIB_DIR}"
  )
endif()

add_executable(example_robot_motion src/example_robot_motion.cpp)
target_link_libraries(example_robot_motion PUBLIC ros_kinematics_utility helper_functions)
if(OPEN3D_FOUND)
  set_target_properties(example_robot_motion PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_RPATH "${OPEN3D_LIB_DIR}"
  )
endif()

add_executable(motion_projection_pcd src/motion_projection_pcd.cpp)
target_link_libraries(motion_projection_pcd PUBLIC
  mesh_projection
  ros_kinematics_utility
  helper_functions
)
if(OPEN3D_FOUND)
  set_target_properties(motion_projection_pcd PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_RPATH "${OPEN3D_LIB_DIR}"
  )
endif()

add_executable(motion_projection_pcd_2 src/motion_projection_pcd_2.cpp)
target_link_libraries(motion_projection_pcd_2 PUBLIC
  mesh_projection
  ros_kinematics_utility
  helper_functions
  mesh_visualization
)
if(OPEN3D_FOUND)
  set_target_properties(motion_projection_pcd_2 PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_RPATH "${OPEN3D_LIB_DIR}"
  )
endif()

add_executable(example_build_adjusancy_map src/example_build_adjusancy_map.cpp)
target_link_libraries(example_build_adjusancy_map PUBLIC
  mesh_projection
  ros_kinematics_utility
  helper_functions
)
if(OPEN3D_FOUND)
  set_target_properties(example_build_adjusancy_map PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_RPATH "${OPEN3D_LIB_DIR}"
  )
endif()

# ============================================================
# Installation
# ============================================================

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install Open3D headers and libraries
if(OPEN3D_FOUND)
  install(DIRECTORY ${Open3D_INCLUDE_DIRS}/ DESTINATION include/Open3D)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Open3D/lib/ DESTINATION lib)
endif()

# Install other directories
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})

# Install targets WITH EXPORT
install(TARGETS
  ros_kinematics_utility
  helper_functions
  mesh_projection
  mesh_visualization
  example_node
  example_robot_motion
  example_generate_mesh
  motion_projection_pcd
  motion_projection_pcd_2
  example_build_adjusancy_map
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ======================================================
# Export the targets for downstream packages
# ======================================================

# Install the targets file
install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Export include directories
ament_export_include_directories(include include/Open3D)

# Export dependencies
ament_export_dependencies(
  rclcpp rcl std_msgs sensor_msgs geometry_msgs
  visualization_msgs Eigen3
  kinematic_and_dynamic_utils
)

ament_package()