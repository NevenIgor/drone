cmake_minimum_required(VERSION 3.8)
project(icm20948_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Создание исполняемого файла
add_executable(icm20948_node src/icm20948_node.cpp)
target_include_directories(icm20948_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(icm20948_node PUBLIC c_std_99 cxx_std_17)

# Правильный способ линковки в ROS 2
ament_target_dependencies(icm20948_node rclcpp sensor_msgs)

# Установка правил для запуска
install(TARGETS icm20948_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
