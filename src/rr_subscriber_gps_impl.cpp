#include <rr_common_plugins/rr_common_subscribers.hpp>

using namespace rr_common_plugins;

/*
 * callback for GPS.
 */
void RrSubscriberGpsImpl::callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received message GPS msg");
    state_->set_gps(*msg);
}