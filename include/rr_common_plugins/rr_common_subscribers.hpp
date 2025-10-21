#ifndef RR_COMMON_PLUGINS__RR_COMMON_SUBSCRIBERS_HPP_
#define RR_COMMON_PLUGINS__RR_COMMON_SUBSCRIBERS_HPP_

#include "rclcpp/logging.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_base/rr_subscriber.hpp"
#include "rr_common_plugins/visibility_control.h"
#include <sensor_msgs/image_encodings.hpp>


namespace rr_common_plugins
{

    /**
     * @class RrSubscriberBase
     * 
     * Provides common routines and variables used in other subscribers.
     */
    class RrSubscriberBase : public rrobot::RrSubscriber
    {
      public:
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
        std::shared_ptr<rrobot::RrStateMaintainer> state_;
        std::string topic_param_ = "not_set";
        std::string topic_ = "not_set";
        std::string frame_id_ = "not_set";

      protected:
        /**
         * @fn initlize
         * 
         * initlize parameters above for the specific subscriber.
         */
        void initialize(std::string topic_param, std::string topic, std::string frame_id)
        {
            topic_param_ = topic_param;
            topic_ = topic;
            frame_id_ = frame_id;
        }
    };

    /**
     * @class RrSubscriberGps
     * @brief implementation of GPS subscription service.
     */
    class RrSubscriberGpsImpl : public RrSubscriberBase, rrobot::RrSubscriberGps
    {
      public:
        RrSubscriberGpsImpl()
        {
            this->initialize("gps-topic", rr_constants::TOPIC_GPS_FIXED, rr_constants::LINK_GPS);
        }

        ~RrSubscriberGpsImpl() = default;

        void callback(const sensor_msgs::msg::NavSatFix::SharedPtr) override;
    };

    /**
     * @class RrSubscriberJoyImpl
     * 
     * The following constants are true for PS4 joysticks not sure if it is true about all controllers.
     * 
     * Note lists for games controllers are split into two parts, sticks which are an anolog component which 
     * gives a value between -1 to 1,  where 0 is centered. 
     * 
     * Buttons which are the various button values on the game controller, these are generally boolean values
     * that can be represented as 1 or 0. Not all buttons are mapped out below.
     */
    class RrSubscriberJoyImpl : public RrSubscriberBase, rrobot::RrSubscriberJoy
    {
      public:
        RrSubscriberJoyImpl()
        {
            this->initialize("joy-topic", rr_constants::TOPIC_JOY, rr_constants::LINK_JOY_PS4);
        }

        ~RrSubscriberJoyImpl() = default;

        void callback(const sensor_msgs::msg::Joy::SharedPtr) override;
    };

    /**
     * Constants relating to battery status.
     * 
     * Battery Status attributes:
     * 
     * * voltage (float32): Battery voltage in volts (V).
     * * temperature (float32): Battery temperature in degrees Celsius (C).
     * * current (float32): Battery current, positive when discharging, negative when charging (amperes).
     * * charge (float32): Charge remaining in ampere-hours (Ah).
     * * capacity (float32): Battery capacity in ampere-hours. For a 3200 mAh battery this would be 3.2, Indicating
     *   3.2 Amp hours.
     * * design_capacity (float32): Original design capacity (Ah).
     * * percentage (float32): State of charge in range [0.0, 1.0].
     * --- 0.0 means the battery is fully discharged (empty).
     * --- 1.0 means the battery is fully charged (100% full).
     * --- Each cell is represented within this task.
     * * power_supply_status (uint8): Status enum (charging, discharging, full, etc.) matching Linux kernel power supply states.
     * * power_supply_health (uint8): Health enum (good, overheat, dead, etc.).
     * * power_supply_technology (uint8): Battery chemistry type (Li-ion, NiMH, etc.).
     * * present (bool): Whether battery is physically present.
     * * cell_voltage (float32[]): Voltage of individual battery cells.
     * * cell_temperature (float32[]): Temperature of individual battery cells.
     * * location (string): Physical location of battery (optional).
     * * serial_number (string): Battery serial number (optional).
     * 
     * Reference: https://docs.ros2.org/foxy/api/sensor_msgs/msg/BatteryState.html
     * 
     * 
     */
    class RrSubscriberBattStateImpl : public RrSubscriberBase, rrobot::RrSubscriberBattState
    {
      public:
        RrSubscriberBattStateImpl()
        {
            this->initialize("batt-topic", rr_constants::TOPIC_BATT_STATE, rr_constants::LINK_BATT_STATE);
        }

        ~RrSubscriberBattStateImpl() = default;

        /**
         *  battery locations, common names.
         */
        const std::string BATT_LOC_BASE = "base";
        const std::string BATT_LOC_LEFT = "left_battery";
        const std::string BATT_LOC_RIGHT = "right_battery";

        void callback(const sensor_msgs::msg::BatteryState::SharedPtr) override;
    };

    class RrImageSubscriberImpl : public RrSubscriberBase, rrobot::RrImageSubscriber
    {
      public:
        RrImageSubscriberImpl()
        {
            this->initialize("rr-image-topic", rr_constants::TOPIC_IMAGE_RAW, rr_constants::LINK_CAM);
        }

        ~RrImageSubscriberImpl() = default;
        void callback(const sensor_msgs::msg::Image::SharedPtr) override;
    };

    class RrImuSubscriberImpl : public RrSubscriberBase, rrobot::RrImuSubscriber
    {
      public:
        RrImuSubscriberImpl()
        {
            this->initialize("rr-imu-topic", rr_constants::TOPIC_IMU, rr_constants::LINK_IMU);
        }

        ~RrImuSubscriberImpl() = default;

        void callback(const sensor_msgs::msg::Imu::SharedPtr) override;
    };

    class RrRangeSubscriberBase : public RrSubscriberBase, rrobot::RrRangesSubscriber
    {
      public:
        void callback(const sensor_msgs::msg::Range::SharedPtr) override;
    };

    /**
     * @class RrRangesSubscriberImpl
     * 
     * Assumes vector of range sensors. For specific sensors this could be done on different topics.
     */
    class RrRangesSubscriberLeft : public RrRangeSubscriberBase
    {
      public:
        RrRangesSubscriberLeft()
        {
            this->initialize("rr-sonic-left-topic", rr_constants::TOPIC_ULTRA_SONIC_LEFT , rr_constants::LINK_ULTRA_SONIC_LEFT);
        }

        ~RrRangesSubscriberLeft() = default;
    };

    class RrRangesSubscriberCenter : public RrRangeSubscriberBase
    {
      public:
        RrRangesSubscriberCenter()
        {
            this->initialize("rr-sonic-center-topic", rr_constants::TOPIC_ULTRA_SONIC_CENTER, rr_constants::LINK_ULTRA_SONIC_CENTER);
        }

        ~RrRangesSubscriberCenter() = default;
    };

    class RrRangesSubscriberRight : public RrRangeSubscriberBase
    {
      public:
        RrRangesSubscriberRight()
        {
            this->initialize("rr-sonic-right-topic", rr_constants::TOPIC_ULTRA_SONIC_RIGHT, rr_constants::LINK_ULTRA_SONIC_RIGHT);
        }

        ~RrRangesSubscriberRight() = default;
    };

} // namespace rr_common_plugins
#endif