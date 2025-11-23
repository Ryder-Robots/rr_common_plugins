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

#include "rr_common_plugins/joy_subscriber_udp_plugin.hpp"

namespace rr_common_plugins::rr_udp_plugins
{
    /**
     * Configure node, let lifecycle node determine if it should be inilized.
     */
    LNI::CallbackReturn RrJoySubscriberUdpPlugin::configure(const lc::State &state, CallbackT cb, rclcpp::Node::SharedPtr node)
    {
        RCLCPP_DEBUG(logger_, "configuring RrJoySubscriberUdpPlugin");
        (void)state;
        cb_ = cb;
        node_ = node;

        // callback is created here, because in activate it can be re-used without redefining.
        plugin_cb_ = std::bind(&RrJoySubscriberUdpPlugin::subscriber_callback, this, _1);
        return LNI::CallbackReturn::SUCCESS;
    }

    /**
     * Create binding routine to callback that will be used.
     */
    LNI::CallbackReturn RrJoySubscriberUdpPlugin::on_activate(const lc::State &state)
    {
        RCLCPP_DEBUG(logger_, "activating RrJoySubscriberUdpPlugin");
        (void)state;
        if (node_ == nullptr || cb_ == nullptr) {
            RCLCPP_ERROR(logger_, "node is not defined, cannot activate");
            return LNI::CallbackReturn::FAILURE;
        }
        rclcpp::SubscriptionOptions options;
        subscription_ = node_->create_subscription<udp_msgs::msg::UdpPacket>("/udp_read", rclcpp::SensorDataQoS(), plugin_cb_);

        return LNI::CallbackReturn::SUCCESS;
    }


    /**
     * called for any UDP packet, but only executes parents node callback if message is a Joy.
     */
    void RrJoySubscriberUdpPlugin::subscriber_callback(const udp_msgs::msg::UdpPacket::UniquePtr &packet)
    {
    }

    /**
     * reset all variables, not that once this routine is called, it will need to re-configured
     */
    LNI::CallbackReturn RrJoySubscriberUdpPlugin::on_cleanup(const lc::State &state)
    {
        (void)state;
        RCLCPP_DEBUG(logger_, "cleaning RrJoySubscriberUdpPlugin");
        if (subscription_ != nullptr) {
            subscription_.reset();
        }
        return LNI::CallbackReturn::SUCCESS;
    }
} // namespace rr_common_plugins::rr_udp_plugins
