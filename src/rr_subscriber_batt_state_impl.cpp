#include "rr_common_plugins/rr_batt_state_subscriber_impl.hpp"

using namespace rr_common_plugins;

/**
 * @fn callback
 * 
 * callback for battery state.
 */
void RrSubscriberBattStateImpl::callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received message battrey state msg");
    state_->set_batt_state(*msg);
}

PLUGINLIB_EXPORT_CLASS(rr_common_plugins::RrSubscriberBattStateImpl, rrobot::RrSubscriberBattState)