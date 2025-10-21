#ifndef RR_COMMON_PLUGINS__RR_STATE_MAINTAINER_HPP_
#define RR_COMMON_PLUGINS__RR_STATE_MAINTAINER_HPP_

#include <array>
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_base/rr_state_maintainer.hpp"
#include "rr_common_plugins/visibility_control.h"

namespace rr_common_plugins
{
    class RrStateMaintainerImpl : public rrobot::RrStateMaintainer
    {
      public:
        /**
         * @fn set_gps has_gps get_gps
         * @brief setters and getters relating to gps
         */
        void set_gps(const sensor_msgs::msg::NavSatFix gps) override;
        bool has_gps() override;
        const sensor_msgs::msg::NavSatFix get_gps() override;

        /**
         * @fn set_joystick has_joystick get_joystick
         * @brief setters and getters relating to joystick
         */
        void set_joystick(const sensor_msgs::msg::Joy joystick) override;
        bool has_joystick() override;
        const sensor_msgs::msg::Joy get_joystick() override;

        /**
         * @fn set_batt_state get_batt_state has_batt_state
         * @brief setters and getters relating to battery state
         */
        void set_batt_state(const sensor_msgs::msg::BatteryState) override;
        bool has_batt_state() override;
        const sensor_msgs::msg::BatteryState get_batt_state() override;

        /**
         * @fn set_image has_image get_image
         * @brief setters and getters relating to images (video stream)
         */
        void set_image(const sensor_msgs::msg::Image) override;
        bool has_image() override;
        const sensor_msgs::msg::Image get_image() override;

        /**
         * @fn set_imu has_imu get_imu
         * @brief setters and getters relating to imu (video stream)
         */
        void set_imu(const sensor_msgs::msg::Imu) override;
        bool has_imu() override;
        const sensor_msgs::msg::Imu get_imu() override;

        void set_range(const sensor_msgs::msg::Range) override;

        bool has_ranges() override;
        const std::vector<sensor_msgs::msg::Range> get_ranges() override;

        const rr_interfaces::msg::FeatureSet get_feature_set() override;

        void init(rclcpp::Logger) override;

      private:
        // variables
        sensor_msgs::msg::NavSatFix gps_;            // GPS
        sensor_msgs::msg::Joy joystick_;             // last command recieved by controller peripheral device
        sensor_msgs::msg::BatteryState batt_state_;  // current battery state
        sensor_msgs::msg::Image img_;                // current image
        sensor_msgs::msg::Imu imu_;                  // IMU
        std::vector<sensor_msgs::msg::Range> ranges_;  // ranges given by ultra-sonic, or other range detecting device
        rr_interfaces::msg::FeatureSet feature_set_; // indicates that feature is present, or has been present
        const std::array<std::string, 3> RANGES_LINKS_ = {rr_constants::LINK_ULTRA_SONIC_CENTER, rr_constants::LINK_ULTRA_SONIC_LEFT, rr_constants::LINK_ULTRA_SONIC_RIGHT};

        // shared mutex to allow multiple readers or one writer
        std::shared_mutex mutex_;
        rclcpp::Logger logger_ = rclcpp::get_logger("state_maintainer");
    };
} // namespace rr_common_plugins

#endif