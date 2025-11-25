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

#include "rr_common_plugins/visibility_control.h"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_base/rr_node_joy_plugin_iface.hpp"
#include "rr_common_plugins/generated/rr_client.pb.h"
#include "sensor_msgs/msg/joy.hpp"
#include "udp_msgs/msg/udp_packet.hpp"
#include <memory>
#include <pluginlib/class_list_macros.hpp>


namespace rr_common_plugins
{
    namespace rr_udp_plugins
    {
        /**
         * @class RrJoySubscriberUdpPlugin
         * @brief Creates subscription with UDP transport drivers, as a plugin.
         *
         * Designed to deserialize joystick messages sent via UDP by transport drivers udp_sender_node.
         * Nodes are expected to use the plugin library to initialize this plugin, and use interfaces::RrNodeJoyPluginIface
         * as the communication interface, thus reducing coupling.
         */
        class RrJoySubscriberUdpPlugin : public rrobots::interfaces::RrNodeJoyPluginIface
        {
          public:
            using CallbackT = std::function<void(const sensor_msgs::msg::Joy &)>;

            explicit RrJoySubscriberUdpPlugin() = default;
            ~RrJoySubscriberUdpPlugin() = default;

            // Delete copy constructor and copy assignment operator
            RrJoySubscriberUdpPlugin(const RrJoySubscriberUdpPlugin &) = delete;
            RrJoySubscriberUdpPlugin &operator=(const RrJoySubscriberUdpPlugin &) = delete;

            /**
             * @fn configure
             * @brief creates callback within plugin.
             *
             * Called during the configure phase of the lifecycle node.
             *
             * @param state nodes previous state when this method is called
             * @param callback to execute on subscription, this must std::function<void(const sensor_msgs::msg::Joy &)>
             * @param node abstract interface of node implementing plugin.
             * @return CallbackReturn returns status result of method.
             */
            [[nodiscard]] LNI::CallbackReturn configure(const lc::State &state, CallbackT cb, std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) override;

            /**
             * @fn on_activate
             * @brief activates the plugin
             *
             * creates subscription and starts listening to ingress UDP packets.
             * 
             * CAVEAT: that deserialization of the inbound packet does not make any assumptions on what
             * values are valid for axes, or buttons,  it is expected that data integrity is performed
             * by the node itself.
             *
             * @param state nodes previous state when this method is called
             * @return CallbackReturn returns status result of method.
             */
            [[nodiscard]] LNI::CallbackReturn on_activate(const lc::State &state) override;

            /**
             * @fn on_deactivate
             * @brief stops the subscriber.
             *
             * stops the subscriber, this can occur before shutdown, or when lifecycle node is adjusting
             * frame frequency.
             *
             * @param state nodes previous state when this method is called
             * @return CallbackReturn returns status result of method.
             */
            [[nodiscard]] LNI::CallbackReturn on_deactivate(const lc::State &state) override;

            /**
             * @fn on_cleanup
             * @brief resets local variables
             *
             * cleans up memory, this is done before shutdown is performed.
             *
             * @param state nodes previous state when this method is called
             * @return CallbackReturn returns status result of method.
             */
            [[nodiscard]] LNI::CallbackReturn on_cleanup(const lc::State &state) override;

            /**
             * @fn subscriber_callback
             * @brief internal calback used for deserialization.
             * @param packet inbound packet
             */
            void subscriber_callback(const udp_msgs::msg::UdpPacket::UniquePtr &packet);

            /**
             * @fn deserialize
             * @brief deserializes inbound packewt and returns stadard Joy message
             */
            sensor_msgs::msg::Joy deserialize(InboundMessage &packet);

          private:
            rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr subscription_ = nullptr;
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ = nullptr;
            CallbackT cb_ = nullptr;
            std::function<void(const udp_msgs::msg::UdpPacket::UniquePtr &packet)> plugin_cb_ = nullptr;
        };
    } // namespace rr_udp_plugins
} // namespace rr_common_plugins

#endif // JOY_SUBSCRIBER_UDP_PLUGIN_HPP