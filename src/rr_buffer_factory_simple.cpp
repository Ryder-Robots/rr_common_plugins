#include "rr_common_plugins/rr_buffer_factory_simple.hpp"

using namespace rr_common_plugins;


/**
 * @fn initialize
 * @brief Creates subscriber callbacks.
 * 
 * Note that according to https://docs.ros.org/en/jazzy/How-To-Guides/Using-callback-groups.html all callbacks that are controlled
 * by the executor, however the factory implements its own callback group for subscriptions to avoid blocks by the nodes default
 * callback group for the timer.
 */
void RrBufferFactorySimple::initialize(rclcpp::Node::SharedPtr ctl, std::shared_ptr<rrobot::RrStateMaintainer> state_manager)
{
    ctl_ = ctl;
    RCLCPP_INFO(ctl_->get_logger(), "creating callback group(s)");
    sub_img_group_ = ctl_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = sub_img_group_;

    RCLCPP_INFO(ctl_->get_logger(), "getting image subscriber params");
    ctl_->declare_parameter(image_subscriber_.get_topic_param(), image_subscriber_.get_topic_default());

    RCLCPP_INFO(ctl_->get_logger(), "creating image subscriber");
    image_subscriber_.set_state_handler(state_manager);
    auto img_callback =  std::bind(&RrImageSubscriberImpl::callback, image_subscriber_, std::placeholders::_1);
    auto img_topic_str = ctl_->get_parameter(image_subscriber_.get_topic_param()).as_string();

    img_subscription_ = ctl_->create_subscription<sensor_msgs::msg::Image>(img_topic_str, rclcpp::SensorDataQoS(), img_callback, options);
}

void RrBufferFactorySimple::tear_down()
{
    RCLCPP_INFO(ctl_->get_logger(), "deconstructing factory");
}

PLUGINLIB_EXPORT_CLASS(rr_common_plugins::RrBufferFactorySimple, rrobot::RrBufFactory)