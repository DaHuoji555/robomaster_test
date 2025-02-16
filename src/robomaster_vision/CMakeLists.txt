cmake_minimum_required(VERSION 3.22.1)
project(robomaster_vision)

set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

# 包含目录
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include/robomaster_vision
)

# video_publisher
add_executable(video_publisher
  src/video_publisher.cpp
)
ament_target_dependencies(video_publisher
  rclcpp sensor_msgs cv_bridge OpenCV
)

# image_processor
# 这里将 image_processor.cpp 与 Light.cpp, KNN.cpp, Armor.cpp, Judge_Light.cpp 一起编译
add_executable(image_processor
  src/image_processor.cpp
  src/Light.cpp
  src/KNN.cpp
  src/Armor.cpp
  src/Judge_Light.cpp
)
ament_target_dependencies(image_processor
  rclcpp sensor_msgs std_msgs OpenCV cv_bridge
)

# result_display
add_executable(result_display
  src/result_display.cpp
)
ament_target_dependencies(result_display
  rclcpp sensor_msgs std_msgs OpenCV cv_bridge
)

# 安装生成的可执行文件
install(TARGETS
  video_publisher
  image_processor
  result_display
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
