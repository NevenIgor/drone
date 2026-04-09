cmake_minimum_required(VERSION 3.8)
project(vo_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Установка launch-файлов (чтобы ros2 launch их видел)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Установка конфигов (опционально)
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
