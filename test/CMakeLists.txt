find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)

set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()

ament_add_gmock(${PROJECT_NAME}_joy_subscriber_udp_plugin_test
    joy_subscriber_udp_plugin_test.cpp
)

target_link_libraries(${PROJECT_NAME}_joy_subscriber_udp_plugin_test  rr_common_plugins)

ament_add_gmock(${PROJECT_NAME}_imu_action_serial_plugin_test
    imu_action_serial_plugin_test.cpp
)

target_link_libraries(${PROJECT_NAME}_imu_action_serial_plugin_test  rr_common_plugins)