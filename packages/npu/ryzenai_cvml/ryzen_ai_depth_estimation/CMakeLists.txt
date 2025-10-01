cmake_minimum_required(VERSION 3.8)
project(ryzen_ai_depth_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Ryzen AI CVML Library
set(AMD_CVML_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../RyzenAI-SW/Ryzen-AI-CVML-Library)
find_package(RyzenAILibrary REQUIRED PATHS ${AMD_CVML_SDK_ROOT})

# Include directories
include_directories(include)
include_directories(${AMD_CVML_SDK_ROOT}/include)
include_directories(${cv_bridge_INCLUDE_DIRS})

# Create executable
add_executable(depth_estimation_node
  src/main.cpp
  src/depth_estimation_node.cpp
)

# Specify libraries to link
ament_target_dependencies(depth_estimation_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)

target_link_libraries(depth_estimation_node
  ${OpenCV_LIBRARIES}
  ${RyzenAILibrary_LIBS}
)

# Install executable
install(TARGETS
  depth_estimation_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
	scripts/video_publisher.py
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
