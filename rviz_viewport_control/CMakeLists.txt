cmake_minimum_required(VERSION 3.8)
project(rviz_viewport_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(viewport_control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Qt5 QUIET COMPONENTS Widgets)
if(Qt5_FOUND)
  set(qt_widgets Qt5::Widgets)
else()
  find_package(Qt6 REQUIRED COMPONENTS Widgets)
  set(qt_widgets Qt6::Widgets)
endif()

set(CMAKE_AUTOMOC ON)

add_library(rviz_viewport_control SHARED
  src/viewport_view_controller.cpp
  include/rviz_viewport_control/viewport_view_controller.hpp)
target_include_directories(rviz_viewport_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rviz_viewport_control
  rclcpp::rclcpp
  pluginlib::pluginlib
  rviz_common::rviz_common
  ${viewport_control_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
)
target_link_libraries(rviz_viewport_control ${qt_widgets})

target_link_options(rviz_viewport_control PRIVATE "LINKER:--no-undefined")

install(TARGETS rviz_viewport_control
        EXPORT export_rviz_viewport_control
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES plugin.xml
        DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_targets(export_rviz_viewport_control)
pluginlib_export_plugin_description_file(rviz_common plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
