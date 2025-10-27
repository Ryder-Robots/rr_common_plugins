find_package(ament_cmake_gmock REQUIRED)

ament_add_gmock(${PROJECT_NAME}_state_maintainer_test
    rr_state_maintainer_test.cpp
)
ament_add_gmock(${PROJECT_NAME}_common_subscriber_test
    rr_common_subscribers_test.cpp
)

target_link_libraries(${PROJECT_NAME}_state_maintainer_test rr_common_plugins)
target_link_libraries(${PROJECT_NAME}_common_subscriber_test rr_common_plugins)