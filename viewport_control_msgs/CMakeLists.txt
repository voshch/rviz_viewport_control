cmake_minimum_required(VERSION 3.8)
project(viewport_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ViewportView.msg"
  "srv/ViewportCapture.srv"
  "srv/ViewportSetProjection.srv"
  "srv/ViewportSetReferenceFrame.srv"
  "srv/ViewportSetView.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  ament_add_gtest(test_messages test/test_messages.cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(test_messages
    "${cpp_typesupport_target}"
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${builtin_interfaces_TARGETS})
endif()

ament_package()
