cmake_minimum_required(VERSION 3.5)

project(ros2_mercury_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(NODES mercury_test_template;mercury_test_sample;mercury_test_subscriber)
foreach(target IN LISTS NODES)
  # 実行ファイルの生成
  add_executable(${target} src/${target}.cpp)
  # 依存パッケージの登録
  ament_target_dependencies(${target} rclcpp geometry_msgs sensor_msgs)
  # 依存ライブラリのリンク
  target_link_libraries(${target} prac_lib)
  # 実行ファイルの設置
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()
