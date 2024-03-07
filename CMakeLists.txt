# Minimum required CMake version
cmake_minimum_required(VERSION 3.11)
# set(CMAKE_TOOLCHAIN_FILE "/vcpkg/scripts/buildsystems/vcpkg.cmake")
# Project name
project(pcdtransfer)


# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(RdKafka REQUIRED)

# find_package(librdkafka REQUIRED)
# Find modern-cpp-kafka using vcpkg
# find_package(unofficial-modern-cpp-kafka CONFIG REQUIRED)

# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};")

# Add subdirectory
add_subdirectory(src/modern-cpp-kafka)
add_subdirectory(src/stpcc)

message("LIBRDKAFKA_LIB: ${LIBRDKAFKA_LIB}")
message("LIBRDKAFKA_INCLUDE_DIR: ${LIBRDKAFKA_INCLUDE_DIR}")

# Include directories
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${LIBRDKAFKA_INCLUDE_DIR}
    src/modern-cpp-kafka/include
    
)




# Add executables
add_executable(pcd_sender src/pcd_sender.cpp) 
add_executable(pcd_receiver src/pcd_receiver.cpp)

add_executable(pcd_sender_kafka src/pcd_sender_kafka.cpp)
add_executable(pcd_receiver_kafka src/pcd_receiver_kafka.cpp)

# Set target dependencies
set(pcd_sender_dependencies
  pcl_conversions
  rcl_interfaces
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
)

set(pcd_receiver_dependencies
  pcl_conversions
  rcl_interfaces
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
)

set(pcd_sender_kafka_dependencies
  pcl_conversions
  rcl_interfaces
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
  )

set(pcd_receiver_kafka_dependencies
  pcl_conversions
  rcl_interfaces
  rclcpp
  rcutils
  sensor_msgs
  std_msgs
  )



ament_target_dependencies(pcd_sender ${pcd_sender_dependencies})
ament_target_dependencies(pcd_receiver ${pcd_receiver_dependencies})

ament_target_dependencies(pcd_sender_kafka ${pcd_sender_kafka_dependencies} ${LIBRDKAFKA_LIBRARY_DIR})
ament_target_dependencies(pcd_receiver_kafka ${pcd_receiver_kafka_dependencies} ${LIBRDKAFKA_LIBRARY_DIR})


# Link the executables with required libraries
set(pcd_common_libraries
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  pcc_common
)

target_link_libraries(pcd_sender ${pcd_common_libraries} )
target_link_libraries(pcd_receiver ${pcd_common_libraries} )
target_link_libraries(pcd_sender_kafka ${pcd_common_libraries} modern-cpp-kafka-api)
target_link_libraries(pcd_receiver_kafka ${pcd_common_libraries} modern-cpp-kafka-api)

target_include_directories(pcd_sender_kafka PUBLIC src/modern-cpp-kafka/include ${LIBRDKAFKA_INCLUDE_DIR})
target_include_directories(pcd_receiver_kafka PUBLIC include src/modern-cpp-kafka/include ${LIBRDKAFKA_INCLUDE_DIR})

# Install executables
install(TARGETS pcd_sender DESTINATION lib/${PROJECT_NAME})
install(TARGETS pcd_receiver DESTINATION lib/${PROJECT_NAME})

install(TARGETS pcd_sender_kafka DESTINATION lib/${PROJECT_NAME})
install(TARGETS pcd_receiver_kafka DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # Find test dependencies
  ament_lint_auto_find_test_dependencies()
endif()

# Prepare the package for ament
ament_package()
