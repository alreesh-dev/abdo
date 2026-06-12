cmake_minimum_required(VERSION 3.8)
project(my_factory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# تثبيت مجلدات المشروع
install(DIRECTORY launch config maps models urdf worlds behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# تثبيت عقدة البايثون وجعلها قابلة للتنفيذ
install(PROGRAMS
  src/laser_merger_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/my_task.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()