#include "rr_common_plugins/rr_image_subscriber_impl.hpp"

using namespace rr_common_plugins;

/**
 * @fn callback
 * @brief updates state with current image sensor output.
 */
void RrImageSubscriberImpl::callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_DEBUG(logger_, "received message image msg");
    state_->set_image(*msg);
}