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

#include <memory>
#include "rr_common_base/rr_node_joy_plugin_iface.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

using namespace std::placeholders;

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

            RrJoySubscriberUdpPlugin() = default;
            ~RrJoySubscriberUdpPlugin() = default;

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
            [[nodiscard]] LNI::CallbackReturn configure(const lc::State &state, CallbackT cb, rclcpp::Node::SharedPtr node) override;

            /**
             * @fn on_activate
             * @brief activates the plugin
             *
             * creates subscription and starts listening to ingress UDP packets
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

        private:
            rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr subscription_ = nullptr;
            rclcpp::Node::SharedPtr node_ = nullptr;
            CallbackT cb_ = nullptr;
            std::function<void(const udp_msgs::msg::UdpPacket::UniquePtr & packet)> plugin_cb_ = nullptr;
            rclcpp::Logger logger_;
            void subscriber_callback(const udp_msgs::msg::UdpPacket::UniquePtr & packet);
        };
    }
}

#endif // JOY_SUBSCRIBER_UDP_PLUGIN_HPP