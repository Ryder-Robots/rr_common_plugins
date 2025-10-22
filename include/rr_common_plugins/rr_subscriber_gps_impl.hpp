#ifndef RR_SUBSCRIBER_GPS_IMPL_HPP
#define RR_SUBSCRIBER_GPS_IMPL_HPP

namespace rr_common_plugins
{
    /**
     * @class RrSubscriberGps
     * @brief implementation of GPS subscription service.
     */
    class RrSubscriberGpsImpl : public rrobot::RrSubscriberGps
    {
      public:
        RrSubscriberGpsImpl(std::shared_ptr<rrobot::RrStateMaintainer> state) : topic_param_("gps-topic"),
                                                                                topic_(rr_constants::TOPIC_GPS_FIXED),
                                                                                frame_id_(rr_constants::LINK_GPS),
                                                                                state_(state)

        {}

        ~RrSubscriberGpsImpl() = default;

        void callback(const sensor_msgs::msg::NavSatFix::SharedPtr) override;

        /**
         * @fn get_topic_param
         * 
         * Returns parameter key so that ROS arguments can be used to define the topic override.
         */
        std::string get_topic_param() override
        {
            return topic_param_;
        }

        /**
         * @fn get_topic_default
         * 
         * Default topic to use, this should be correct in most cases.
         */
        std::string get_topic_default() override
        {
            return topic_;
        }

        /**
         * @fn set_logger
         * 
         * Allows factory to override the logger.
         */
        void set_logger(rclcpp::Logger logger) override
        {
            logger_ = logger;
        }

        /**
         * @fn set_state_handler
         * 
         * sets common state maintainer variable.
         */
        void set_state_handler(std::shared_ptr<rrobot::RrStateMaintainer> state) override
        {
            state_ = state;
        }

      protected:
        rclcpp::Logger logger_ = rclcpp::get_logger("subscriber");
        std::string topic_param_;
        std::string topic_;
        std::string frame_id_;
        std::shared_ptr<rrobot::RrStateMaintainer> state_;
    };
} // namespace rr_common_plugins

#endif // RR_SUBSCRIBER_GPS_IMPL_HPP