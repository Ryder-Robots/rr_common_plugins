#include "rr_common_plugins/rr_subscriber_joy_impl.hpp"

using namespace rr_common_plugins;

/**
 * @fn callback
 * 
 * callback for Joystick.
 */
void RrSubscriberJoyImpl::callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received message Joy msg");
    state_->set_joystick(*msg);
}

PLUGINLIB_EXPORT_CLASS(rr_common_plugins::RrSubscriberJoyImpl, rrobot::RrSubscriberJoy)