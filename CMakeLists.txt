cmake_minimum_required(VERSION 3.20)
project(rr_common_plugins)

include(FetchContent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rr_common_base REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(udp_msgs REQUIRED)
find_package(Protobuf REQUIRED)

FetchContent_Declare(
  rr_proto
  URL https://github.com/Ryder-Robots/rr_proto/releases/download/0.0.1-DEVEL/proto.tar.gz
  URL_HASH SHA256=c705cb1d1e37050a4a00e1cf3268c9255f2c68b5cead51c33bfa4f74946a66b0
  SOURCE_SUBDIR proto
)
FetchContent_MakeAvailable(rr_proto)

set(PROTO_FILES ${rr_proto_SOURCE_DIR}/inbound.proto)

# # Generate source and header from proto files 
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS 
  ${PROTO_FILES}
  PROTOC_OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated"
)

add_library(rr_common_plugins src/joy_subscriber_udp_plugin.cpp)
add_library(rr_common_plugins::rr_common_plugins ALIAS rr_common_plugins)
target_compile_features(rr_common_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rr_common_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  rr_common_plugins PUBLIC
  ${rclcpp_TARGETS}
  ${pluginlib_TARGETS}
  ${rr_common_base_TARGETS}
  ${rr_interfaces_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${udp_msgs_TARGETS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rr_common_plugins PRIVATE "RR_COMMON_PLUGINS_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS rr_common_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  rr_common_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
