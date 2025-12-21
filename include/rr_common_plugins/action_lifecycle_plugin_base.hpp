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


#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rr_common_base/rr_action_plugin_iface.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_plugins/generated/rr_serial.pb.h"
#include <memory>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace rr_common_plugins
{
    /**
     * @class RRActionPluginBase
     * 
     * @brief provides lifecycle methods for plugin actions
     */
    class RRActionPluginBase
    {
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        using State = rclcpp_lifecycle::State;
        using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
        using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;
        using LifecyclePublisher = rclcpp_lifecycle::LifecyclePublisher<UInt8MultiArray>;
        using Subscription = rclcpp::Subscription<UInt8MultiArray>;
        using RRActionStatusE = rr_constants::rr_action_status_t;
        using RROpCodeE = rr_constants::rr_op_code_t;
        using SerialResponse = org::ryderrobots::ros2::serial::Response;
        using SerialRequest = org::ryderrobots::ros2::serial::Request;

      public:
        explicit RRActionPluginBase(RROpCodeE op_code, std::shared_ptr<std::mutex> mutex) : op_code_(op_code), mutex_(mutex) {}
        ~RRActionPluginBase() = default;

        [[nodiscard]] CallbackReturn on_configure(const State &state, LifecycleNode::SharedPtr node);

        [[nodiscard]] CallbackReturn on_activate(const State &state);

        [[nodiscard]] CallbackReturn on_deactivate(const State &state);

        [[nodiscard]] CallbackReturn on_cleanup(const State &state);

        LifecyclePublisher::SharedPtr publisher_ = nullptr;

        void set_status(RRActionStatusE status);

        RRActionStatusE get_status();

        bool is_res_avail();

        SerialResponse get_res();

        bool publish(SerialRequest req);

      private:
        Subscription::SharedPtr subscription_ = nullptr;
        const std::string WRITE_TOPIC_ = "/serial_write";
        const std::string READ_TOPIC_ = "/serial_read";
        rclcpp::Logger logger_ = rclcpp::get_logger("ImuActionSerialPlugin");
        void set_res(SerialResponse res);

        void subscriber_cb(const UInt8MultiArray::UniquePtr &packet);
        bool buf_complete_ = false;
        bool res_avail_ = false;
        std::string buffer_;

        const uint8_t TERM_CHAR = 0x1E;

        RRActionStatusE status_ = RRActionStatusE::ACTION_STATE_PREPARING;
        RROpCodeE op_code_;
        SerialResponse res_;

        std::shared_ptr<std::mutex> mutex_;
    };
} // namespace rr_common_plugins