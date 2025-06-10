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
find_package(visualization_msgs REQUIRED)
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
  src # For PlyWriter.h if included as "PlyWriter.h" from pointcloud_accumulator_node.cpp
)

# Define a list for external libraries
set(EXTERNAL_LIBS ${PCL_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)
if(OpenMP_FOUND AND OpenMP_CXX_LIBRARIES)
  list(APPEND EXTERNAL_LIBS ${OpenMP_CXX_LIBRARIES})
elseif(OpenMP_FOUND AND NOT OpenMP_CXX_LIBRARIES)
  # Fallback if OpenMP_CXX_LIBRARIES is not set
  # This might happen if OpenMP is header-only or only provides flags.
  # CMake sets OpenMP_FOUND if flags are found.
  # If using modern CMake imported targets for OpenMP (e.g. OpenMP::OpenMP_CXX)
  # that would be preferred, but for plain linking, we rely on OpenMP_CXX_LIBRARIES.
  # If it's empty, nothing is added, which is correct.
  # message(STATUS "OpenMP_CXX_LIBRARIES not set, but OpenMP_FOUND. Flags should be applied.")
endif()

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
  visualization_msgs
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

# Adaptive Parameter Manager Library
add_library(adaptive_parameter_manager_lib
  src/adaptive_parameter_manager.cpp
)

ament_target_dependencies(adaptive_parameter_manager_lib
  "rclcpp"
  "std_msgs"
)

target_include_directories(adaptive_parameter_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src> # For adaptive_parameter_manager.h
  $<INSTALL_INTERFACE:include> # Assuming headers might be installed to include/
)

# Adaptive Parameter Manager Node Executable
add_executable(adaptive_parameter_manager_node
  src/adaptive_parameter_manager_node_main.cpp
)

ament_target_dependencies(adaptive_parameter_manager_node
  "rclcpp" # For main function's rclcpp::init/spin/shutdown
)

target_link_libraries(adaptive_parameter_manager_node
  adaptive_parameter_manager_lib
  # rclcpp::rclcpp # Provided by ament_target_dependencies
)

# No direct PCL, OpenCV, or Eigen linking needed for adaptive_parameter_manager_node based on its current code.
# If it were to process point clouds directly, these would be added.
if(OpenMP_FOUND) # Though unlikely to be beneficial for this node's logic
  target_link_libraries(adaptive_parameter_manager_node OpenMP::OpenMP_CXX) # This might be redundant if lib has it
endif()

# PointCloud Accumulator Node
add_executable(pointcloud_accumulator_node
  src/pointcloud_accumulator_node.cpp
  src/PlyWriter.cpp
)
ament_target_dependencies(pointcloud_accumulator_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(pointcloud_accumulator_node
  ${PCL_LIBRARIES}
)
# Ensure C++17 features are compiled for the node (project globally sets C++17)
target_compile_features(pointcloud_accumulator_node PRIVATE cxx_std_17)


# Install executables
install(TARGETS
  livox_repub
  loam_scanRegistration
  loam_scanRegistration_horizon
  loam_laserMapping
  adaptive_parameter_manager_node
  adaptive_parameter_manager_lib # Added library
  pointcloud_accumulator_node # Added new node
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Add the test executable for AdaptiveParameterManager
  # The explicit add_executable is removed. ament_add_gtest will create the target.

  # Add GTest and specify all sources needed for the test executable
  ament_add_gtest(test_adaptive_parameter_manager
    test/test_adaptive_parameter_manager.cpp
    # src/adaptive_parameter_manager.cpp # No longer needed, link the library instead
    # src/adaptive_parameter_manager_types.h # .h files are not typically listed here, handled by include_directories
  )

  # Link against necessary libraries for the test executable
  # rclcpp and std_msgs dependencies for the test code itself.
  # The functionality of AdaptiveParameterManager comes from the linked library.
  ament_target_dependencies(test_adaptive_parameter_manager
    rclcpp
    std_msgs
  )

  target_link_libraries(test_adaptive_parameter_manager
    adaptive_parameter_manager_lib
  )

  # Ensure the test executable can find headers from src/ (where adaptive_parameter_manager.h is)
  target_include_directories(test_adaptive_parameter_manager
    PRIVATE
    src # To find adaptive_parameter_manager.h and types.h
    # Add other include directories if needed
  )

  # If adaptive_parameter_manager.cpp itself has OpenMP dependencies,
  # they might need to be linked here too, but it's unlikely for this specific class.
  # if(OpenMP_FOUND)
  #   target_link_libraries(test_adaptive_parameter_manager OpenMP::OpenMP_CXX)
  # endif()

endif()

# ament_package() call (must be called after all targets are defined)
ament_package()
