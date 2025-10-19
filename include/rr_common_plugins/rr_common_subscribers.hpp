#ifndef RR_COMMON_PLUGINS__RR_COMMON_SUBSCRIBERS_HPP_
#define RR_COMMON_PLUGINS__RR_COMMON_SUBSCRIBERS_HPP_

#include "rr_common_base/rr_subscriber.hpp"
#include "rr_common_plugins/visibility_control.h"
#include "rclcpp/logging.hpp"
#include <sensor_msgs/image_encodings.hpp>

namespace rr_common_plugins
{
    /**
     * @class RrSubscriberGps
     * @brief implementation of GPS subscription service.
     */
    class RrSubscriberGpsImpl : public rrobot::RrSubscriberGps
    {
      public:
        std::string get_topic_param() override
        {
            return topic_param_;
        }

        std::string get_topic_default() override
        {
            return topic_;
        }

        void set_logger(rclcpp::Logger logger) override
        {
            logger_ = logger;
        }

        void set_state_handler(std::shared_ptr<rrobot::RrStateMaintainer> state)
        {
            state_ = state;
        }

        void callback(const sensor_msgs::msg::NavSatFix::SharedPtr) override;

        const std::string topic_param_ = "gps-topic";
        const std::string topic_ = "/fix";
        const std::string frame_id_ = "gps_link";

      private:
        rclcpp::Logger logger_ = rclcpp::get_logger("gps_subscriber");
        std::shared_ptr<rrobot::RrStateMaintainer>  state_;
    };
} // namespace rr_common_plugins
#endif