cmake_minimum_required(VERSION 3.8)
project(promobot_tf_library)
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/jazzy")

# Настройки C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Опции компиляции
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Создание библиотеки
add_library(coordinate_transformer SHARED
  src/coordinate_transformer.cpp
)

# Настройки библиотеки
target_include_directories(coordinate_transformer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Зависимости библиотеки (используем ключевой синтаксис)
ament_target_dependencies(coordinate_transformer PUBLIC
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  rcl_interfaces
  ament_index_cpp
)

# Явная линковка с yaml-cpp (ключевой синтаксис)
target_link_libraries(coordinate_transformer
  PUBLIC yaml-cpp
)

# Исполняемый файл примера
add_executable(transformer_example_node 
  src/transformer_example_node.cpp
)

# Зависимости примера (только пакеты ROS2)
ament_target_dependencies(transformer_example_node
  rclcpp
  geometry_msgs
)

# Линковка примера с библиотекой и yaml-cpp
target_link_libraries(transformer_example_node
  coordinate_transformer
  yaml-cpp
)

# Установка библиотеки
install(TARGETS coordinate_transformer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Установка заголовков
install(DIRECTORY include/
  DESTINATION include
)

# Установка примера
install(TARGETS transformer_example_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка YAML-файла
install(FILES transforms.yaml
  DESTINATION share/${PROJECT_NAME}
)

# Экспорт для других пакетов
ament_export_include_directories(include)
ament_export_libraries(coordinate_transformer)
ament_export_dependencies(
  rclcpp 
  rclcpp_components 
  tf2 
  tf2_ros 
  geometry_msgs 
  yaml-cpp
  rcl_interfaces 
  ament_index_cpp
)

# Регистрация компонента
rclcpp_components_register_nodes(coordinate_transformer 
  "promobot_tf_library::CoordinateTransformer"
)


ament_package()

