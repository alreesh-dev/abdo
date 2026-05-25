cmake_minimum_required(VERSION 3.8)
project(ira_laser_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) # تم تحديثه إلى 17 ليتوافق مع مكتبات TF2 الحديثة
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) 

add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)

# تم إضافة جميع الاعتماديات هنا ليتعرف عليها الكومبيلر بدقة
ament_target_dependencies(laserscan_multi_merger
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# تثبيت الملف التنفيذي
install(TARGETS laserscan_multi_merger
  DESTINATION lib/${PROJECT_NAME})

ament_package()