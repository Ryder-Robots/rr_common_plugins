#include <rr_common_plugins/rr_common_subscribers.hpp>

using namespace rr_common_plugins;

/*
 * callback for GPS.
 */
//TODO: fix the following
void RrSubscriberGpsImpl::callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // RCLCPP_DEBUG(logger_, "received message GPS msg");
    // state_->set_gps(*msg);
}

void RrSubscriberJoyImpl::callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received message Joy msg");
    state_->set_joystick(*msg);
}

void RrSubscriberBattStateImpl::callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received message battrey state msg");
    state_->set_batt_state(*msg);
}

void RrImageSubscriberImpl::callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received message image msg");
    state_->set_image(*msg);
}

void RrImuSubscriberImpl::callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received IMU msg");
    state_->set_imu(*msg);
}

void RrRangeSubscriberBase::callback(const sensor_msgs::msg::Range::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received ranges msg");
    state_->set_range(*msg);
}