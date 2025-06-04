cmake_minimum_required(VERSION 3.8)
project(livox_mapping)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True) # Ensure C++17 is used, fail if not available
set(CMAKE_CXX_EXTENSIONS OFF) # Disable compiler-specific extensions like -std=gnu++17

# Compiler flags - keep relevant ones, remove old std flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -Wno-unused-local-typedefs")

# Find OpenMP
find_package(OpenMP QUIET)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  # For modern CMake, OpenMP::OpenMP_C and OpenMP::OpenMP_CXX might be available as targets
  # message(STATUS "OpenMP C flags: ${OpenMP_C_FLAGS}")
  # message(STATUS "OpenMP CXX flags: ${OpenMP_CXX_FLAGS}")
else()
  message(WARNING "OpenMP not found. Proceeding without OpenMP support.")
endif()

# Find ament_cmake and required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED) # If you have Python nodes or scripts
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # For tf2::fromMsg, tf2::toMsg with geometry_msgs
# find_package(pcl_ros REQUIRED) # REMOVED

# Find external libraries
# PCL Setup Block (modified for older PCL style)
find_package(PCL REQUIRED)
message(STATUS "PCL_FOUND: ${PCL_FOUND}")
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")
message(STATUS "PCL_VERSION: ${PCL_VERSION}")

find_package(Eigen3 REQUIRED CONFIG) # Use CONFIG mode for modern Eigen3
find_package(pcl_conversions REQUIRED) # ADDED
find_package(livox_ros_driver2 REQUIRED) # Assuming this is a ROS 2 package
find_package(OpenCV REQUIRED)

# Include directories
# Add 'include' directory if it exists for package-specific headers
include_directories(
  include 
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Executables
add_executable(livox_repub src/livox_repub.cpp)
ament_target_dependencies(livox_repub
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions # ADDED
  livox_ros_driver2
)
target_link_libraries(livox_repub 
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen # Modern Eigen uses imported targets
)
if(OpenMP_FOUND)
  target_link_libraries(livox_repub OpenMP::OpenMP_CXX)
endif()

add_executable(loam_scanRegistration src/scanRegistration.cpp)
ament_target_dependencies(loam_scanRegistration
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs # ADDED
  pcl_conversions # ADDED
  livox_ros_driver2
)
target_link_libraries(loam_scanRegistration
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
if(OpenMP_FOUND)
  target_link_libraries(loam_scanRegistration OpenMP::OpenMP_CXX)
endif()

add_executable(loam_scanRegistration_horizon src/scanRegistration_horizon.cpp)
ament_target_dependencies(loam_scanRegistration_horizon
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs # ADDED
  pcl_conversions # ADDED
  livox_ros_driver2
)
target_link_libraries(loam_scanRegistration_horizon
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
if(OpenMP_FOUND)
  target_link_libraries(loam_scanRegistration_horizon OpenMP::OpenMP_CXX)
endif()

add_executable(loam_laserMapping src/laserMapping.cpp)
ament_target_dependencies(loam_laserMapping
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  pcl_conversions # ADDED
  livox_ros_driver2
)
target_include_directories(loam_laserMapping PRIVATE ${tf2_geometry_msgs_INCLUDE_DIRS})
target_link_libraries(loam_laserMapping
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
if(OpenMP_FOUND)
  target_link_libraries(loam_laserMapping OpenMP::OpenMP_CXX)
endif()

# Install executables
install(TARGETS
  livox_repub
  loam_scanRegistration
  loam_scanRegistration_horizon
  loam_laserMapping
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (assuming they are in a 'launch' directory)
# Create a dummy launch directory for now if it doesn't exist, or remove this if no launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  # Add PATTERN or FILES_MATCHING if you want to exclude/include specific files
  # OPTIONAL # This makes the install command not fail if the directory doesn't exist
)

# Install other directories like config or params if they exist
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# ament_package() call (must be called after all targets are defined)
ament_package()
