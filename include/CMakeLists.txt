 # Configure external include files for plugins
 
 configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/visibility_control.h.in 
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/visibility_control.h
  @ONLY
)

 configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/rr_state_maintainer_impl.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/rr_state_maintainer_impl.hpp
  @ONLY
)

set(FILE_NAME_L "rr_subscriber_gps_impl;rr_subscriber_joy_impl;rr_batt_state_subscriber_impl;rr_image_subscriber_impl;rr_imu_subscriber_impl;rr_range_left_subscriber_impl;rr_range_center_subscriber_impl;rr_range_right_subscriber_impl")
set(CLASS_NAME_L "RrSubscriberGpsImpl;RrSubscriberJoyImpl;RrSubscriberBattStateImpl;RrImageSubscriberImpl;RrImuSubscriberImpl;RrRangesSubscriberLeft;;RrRangesSubscriberRight")
set(TOPIC_PARAM_L  "gps-topic;joy-topic;batt-topic;rr-image-topic;rr-imu-topic;rr-sonic-left-topic;;rr-sonic-right-topic")
set(TOPIC_L  "rr_constants::TOPIC_GPS_FIXED;rr_constants::TOPIC_JOY;rr_constants::TOPIC_BATT_STATE;rr_constants::TOPIC_IMAGE_RAW;rr_constants::TOPIC_IMU;rr_constants::TOPIC_ULTRA_SONIC_LEFT;rr_constants::TOPIC_ULTRA_SONIC_CENTER;rr_constants::TOPIC_ULTRA_SONIC_RIGHT")
set(FRAME_ID_L  "rr_constants::LINK_GPS;rr_constants::LINK_JOY_PS4;rr_constants::LINK_BATT_STATE;rr_constants::LINK_CAM;rr_constants::LINK_IMU;rr_constants::LINK_ULTRA_SONIC_LEFT;rr_constants::LINK_ULTRA_SONIC_CENTER;rr_constants::LINK_ULTRA_SONIC_RIGHT")
set(MSG_CLASS_L  "sensor_msgs::msg::NavSatFix::SharedPtr;sensor_msgs::msg::Joy::SharedPtr;sensor_msgs::msg::BatteryState::SharedPtr;sensor_msgs::msg::Image::SharedPtr;sensor_msgs::msg::Imu::SharedPtr;sensor_msgs::msg::Range::SharedPtr;sensor_msgs::msg::Range::SharedPtr;sensor_msgs::msg::Range::SharedPtr")
set(BASE_CLASS_L "RrSubscriberGps;RrSubscriberJoy;RrSubscriberBattState;RrImageSubscriber;RrImuSubscriberImpl;RrRangeSubscriberBase;RrRangeSubscriberBase;RrRangeSubscriberBase")

foreach(FILE_NAME 
  CLASS_NAME 
  TOPIC_PARAM 
  TOPIC
  FRAME_ID
  MSG_CLASS
  BASE_CLASS
IN ZIP_LISTS FILE_NAME_L CLASS_NAME_L TOPIC_PARAM_L TOPIC_L FRAME_ID_L MSG_CLASS_L BASE_CLASS_L)
  set(OUT "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${FILE_NAME}.hpp")
  string(TOUPPER "${FILE_NAME}" FILE_NAME_UC)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/rr_subscriber_impl.hpp.in
    ${OUT}
    @ONLY
  )
endforeach()
