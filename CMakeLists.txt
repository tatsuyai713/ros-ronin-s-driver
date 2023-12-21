cmake_minimum_required(VERSION 3.5)
project(ros_ronin_s_driver)

# 必要なパッケージを探す
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_kvaser_can_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# カスタムメッセージのインクルード
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DJIRoninSStatus.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# インクルードディレクトリをエクスポート
ament_export_include_directories(include)
include_directories(include)

# パッケージをビルドするための設定
ament_package()

# ビルドに必要なライブラリを指定
add_executable(ronin_s_driver src/ronin_s_driver.cpp)
ament_target_dependencies(ronin_s_driver rclcpp std_msgs geometry_msgs ros_kvaser_can_driver)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ronin_s_driver "${cpp_typesupport_target}") 

install(TARGETS ronin_s_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
)

