#include "rr_common_plugins/rr_subscriber_gps_impl.hpp"

using namespace rr_common_plugins;

/**
 * @fn callback
 * 
 * callback for GPS.
 */
void RrSubscriberGpsImpl::callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received message GPS msg");
    state_->set_gps(*msg);
}

PLUGINLIB_EXPORT_CLASS(rr_common_plugins::RrSubscriberGpsImpl, rrobot::RrSubscriberGps)
