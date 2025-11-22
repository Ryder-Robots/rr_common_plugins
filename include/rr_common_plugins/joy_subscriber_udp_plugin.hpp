// Copyright (c) 2025 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef JOY_SUBSCRIBER_UDP_PLUGIN_HPP
#define JOY_SUBSCRIBER_UDP_PLUGIN_HPP

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rr_common_base/rr_node_joy_plugin_iface.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace rrobots
{
    namespace rr_joy_plugin
    {
        /**
         * @class RrJoySubscriberUdpPlugin
         * @brief Creates subscription with UDP transport drivers, as a plugin.
         * 
         * Designed to deserialize joystick messages send via UDP by transport drivers udp_sendor_node.
         * Nodes are expected to use plugin library initilize this plugin, and use interfaces::RrNodeJoyPluginIface
         * as the communication interface, thus reducing coupling.
         */
        class RrJoySubscriberUdpPlugin : public interfaces::RrNodeJoyPluginIface
        {
          public:
            using CallbackTypeP = std::function<void(const sensor_msgs::msg::Joy &)>;

            RrJoySubscriberUdpPlugin() {}

            ~RrJoySubscriberUdpPlugin() = default;

            /**
             * @fn configure
             * @brief creates callback within plugin.
             * 
             * expected to be called during the configure section of a lifecycle node.
             * 
             * @param state nodes previous state when this method is called
             * @param callback to execute on subscription, this must std::function<void(const sensor_msgs::msg::Joy &)>
             * @return CallbackReturn returns status result of method.
             */
            [[nodiscard]] LNI::CallbackReturn configure(const lc::State &state, CallbackTypeP cb) override;

            [[nodiscard]] LNI::CallbackReturn on_activate(const lc::State &state) override;

            [[nodiscard]] LNI::CallbackReturn on_deactivate(const lc::State &state) override;

            [[nodiscard]] LNI::CallbackReturn on_cleanup(const lc::State &state) override;

          private:
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
        };
    } // namespace rr_joy_plugin
} // namespace rrobots

#endif // JOY_SUBSCRIBER_PLUGIN_HPP