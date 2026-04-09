cmake_minimum_required(VERSION 3.8)
project(msp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Создаём исполняемый файл
add_executable(msp_bridge_node src/msp_bridge.cpp)
target_include_directories(msp_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(msp_bridge_node PUBLIC c_std_99 cxx_std_17)

# Правильная линковка зависимостей в ROS 2
ament_target_dependencies(msp_bridge_node rclcpp nav_msgs)

# Установка для запуска через ros2 run
install(TARGETS msp_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
