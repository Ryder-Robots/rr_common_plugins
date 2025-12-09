cmake_minimum_required(VERSION 3.20)
project(rr_common_plugins)

include(FetchContent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "C compiler: ${CMAKE_C_COMPILER}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Toolchain file: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "C compiler ID: ${CMAKE_C_COMPILER_ID}")
message(STATUS "C++ compiler ID: ${CMAKE_CXX_COMPILER_ID}")

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
  URL https://github.com/Ryder-Robots/rr_proto/archive/refs/tags/0.0.7-DEVEL.tar.gz
  GIT_TAG 0.0.7-DEVEL
  SOURCE_SUBDIR proto
)
FetchContent_MakeAvailable(rr_proto)

set(PROTOC_OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/rr_common_plugins/generated")
file(MAKE_DIRECTORY "${PROTOC_OUT_DIR}")
set(PROTO_FILES ${rr_proto_SOURCE_DIR}/proto/rr_client.proto)

message(STATUS "PROTPROTOC_OUT_DIR_FILES: ${PROTOC_OUT_DIR}")
message(STATUS "PROTO_FILES: ${PROTO_FILES}")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS 
  ${PROTO_FILES}
  PROTOC_OUT_DIR "${PROTOC_OUT_DIR}"
)

add_library(rr_common_plugins 
  src/joy_subscriber_udp_plugin.cpp
  ${PROTO_SRCS} 
  ${PROTO_HDRS})

add_library(rr_common_plugins::rr_common_plugins ALIAS rr_common_plugins)
target_compile_features(rr_common_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rr_common_plugins PUBLIC
  ${Protobuf_INCLUDE_DIRS} 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_definitions(rr_common_plugins
  PRIVATE  rr_common_plugins_EXPORTS)  
target_link_libraries(
  rr_common_plugins PUBLIC
  ${rclcpp_TARGETS}
  ${pluginlib_TARGETS}
  ${rr_common_base_TARGETS}
  ${rr_interfaces_TARGETS}
  ${lifecycle_msgs_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${udp_msgs_TARGETS}
  protobuf::libprotobuf
)

message(STATUS "PROTO_HDRS: ${PROTO_HDRS}")

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rr_common_plugins PRIVATE "RR_COMMON_PLUGINS_BUILDING_LIBRARY")

install(
  DIRECTORY include/ ${CMAKE_CURRENT_SOURCE_DIR}/include
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
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
  "include/${PROJECT_NAME}/generated"
)
ament_export_libraries(
  rr_common_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rr_common_base plugins.xml)
ament_package()
