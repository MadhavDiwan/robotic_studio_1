cmake_minimum_required(VERSION 3.5)
project(overlay_package)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

# Add your executable
add_executable(overlay_node src/overlay_node.cpp)

# Link against OpenCV and rclcpp
target_link_libraries(overlay_node ${OpenCV_LIBS} rclcpp::rclcpp)

install(TARGETS overlay_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
ament_package()
