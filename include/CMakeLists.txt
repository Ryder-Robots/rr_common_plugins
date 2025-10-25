 # Configure external include files for plugins
 
 configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/visibility_control.h.in 
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/visibility_control.h
  @ONLY
)

set(FILE_NAME "rr_subscriber_gps_impl")
string(TOUPPER "${FILE_NAME}" FILE_NAME_UC)
set(CLASS_NAME "RrSubscriberGpsImpl")
set(TOPIC_PARAM  "gps-topic")
set(TOPIC  "rr_constants::TOPIC_GPS_FIXED")
set(FRAME_ID  "rr_constants::LINK_GPS")
set(MSG_CLASS  "sensor_msgs::msg::NavSatFix::SharedPtr")
set(OUT "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${FILE_NAME}.hpp")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/rr_subscriber_impl.hpp.in
  ${OUT}
  @ONLY
)

# install(
#   DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}
# )