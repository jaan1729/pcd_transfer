cmake_minimum_required(VERSION 3.5)
project(pcd_compresser)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_subdirectory(src/stpcc)

# Include directories
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# Link the executable with required libraries


# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
# ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

# Add executables
add_executable(pcd_encoder_ros src/pcd_encoder_ros.cpp) 
ament_target_dependencies(pcd_encoder_ros
  "pcl_conversions"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  )

target_link_libraries(pcd_encoder_ros
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  pcc_common
)

# Install executables
install(TARGETS pcd_encoder_ros
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Find test dependencies
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Prepare the package for ament
ament_package()
