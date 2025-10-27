#include "rr_common_plugins/rr_buffer_factory_simple.hpp"

using namespace rr_common_plugins;


/**
 * Creates simple callbacks.
 */
void RrBufferFactorySimple::initialize(rclcpp::Node::SharedPtr ctl, std::shared_ptr<rrobot::RrStateMaintainer> state_manager)
{
    ctl_ = ctl;
    RCLCPP_INFO(ctl_->get_logger(), "creating callback group(s)");
    auto cg = ctl_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;

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