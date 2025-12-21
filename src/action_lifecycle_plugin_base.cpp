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

#include "rr_common_plugins/action_lifecycle_plugin_base.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;
using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;
using LifecyclePublisher = rclcpp_lifecycle::LifecyclePublisher<UInt8MultiArray>;
using Subscription = rclcpp::Subscription<UInt8MultiArray>;
using RRActionStatusE = rr_constants::rr_action_status_t;
using SerialResponse = org::ryderrobots::ros2::serial::Response;
using SerialRequest = org::ryderrobots::ros2::serial::Request;
using Time = builtin_interfaces::msg::Time;

namespace rr_common_plugins
{
    CallbackReturn RRActionPluginBase::on_configure(const State &state, LifecycleNode::SharedPtr node)
    {
        (void)state;
        auto topic_names_and_types = node->get_topic_names_and_types();
        auto found = false;
        timestamp_ = clock_.now();

        for (std::string t : {"/serial_read", "/serial_write"}) {
            for (const auto &[topic_name, type_list] : topic_names_and_types) {
                if (topic_name == t) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                RCLCPP_ERROR(logger_, "topics are not available");
                return CallbackReturn::FAILURE;
            }
            found = false;
        }

        // // create subscriptions
        auto plugin_cb_ = std::bind(&RRActionPluginBase::subscriber_cb, this, std::placeholders::_1);
        rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
        publisher_ = node->create_publisher<UInt8MultiArray>(WRITE_TOPIC_, rclcpp::SensorDataQoS(), options);
        subscription_ = node->create_subscription<UInt8MultiArray>(READ_TOPIC_, rclcpp::SensorDataQoS(), plugin_cb_);
        buffer_.clear();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RRActionPluginBase::on_activate(const State &state)
    {
        (void)state;
        // clear the buffer for safety
        set_status(RRActionStatusE::ACTION_STATE_PREPARING);
        publisher_->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RRActionPluginBase::on_deactivate(const State &state)
    {
        (void)state;
        set_status(RRActionStatusE::ACTION_STATE_PREPARING);
        subscription_ = nullptr;
        publisher_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RRActionPluginBase::on_cleanup(const State &state)
    {
        (void)state;
        return CallbackReturn::SUCCESS;
    }

    void RRActionPluginBase::set_status(RRActionStatusE status)
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        status_ = status;
        buf_complete_ = true;
        buffer_.clear();
    }

    RRActionStatusE RRActionPluginBase::get_status()
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        return status_;
    }

    void RRActionPluginBase::set_res(SerialResponse res)
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        timestamp_ = clock_.now();
        res_ = res;
        res_avail_ = true;
    }

    Time RRActionPluginBase::get_time_stamp()
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        return timestamp_;
    }

    bool RRActionPluginBase::is_res_avail()
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        return res_avail_;
    }

    SerialResponse RRActionPluginBase::get_res()
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        res_avail_ = false;
        return res_;
    }

    bool RRActionPluginBase::publish(SerialRequest req)
    {
        
        std::string serialized_data;
        // done for safety
        serialized_data.clear();
        if (!req.SerializeToString(&serialized_data)) {
            RCLCPP_ERROR(logger_, "Failed to serialize request");
            set_status(RRActionStatusE::ACTION_STATE_FAIL);
            return false;
        }
        serialized_data.push_back(TERM_CHAR);
        UInt8MultiArray msg;
        msg.data.resize(serialized_data.size());
        std::memcpy(msg.data.data(), serialized_data.data(), serialized_data.size());

        const std::lock_guard<std::mutex> lock(*mutex_);
        publisher_->publish(msg);
        return true;
    }

    void RRActionPluginBase::subscriber_cb(const UInt8MultiArray::UniquePtr &packet)
    {
        RCLCPP_DEBUG(logger_, "subscriber called");
        if (buf_complete_) {
            set_status(RRActionStatusE::ACTION_STATE_PREPARING);
        }

        if (!packet || packet->data.empty()) {
            RCLCPP_DEBUG(logger_, "recieved empty packet call");
            return;
        }

        if (buffer_.size() + packet->data.size() > BUFSIZ) {
            RCLCPP_ERROR(logger_, "packet too large");
            set_status(RRActionStatusE::ACTION_STATE_FAIL);
        }

        auto term_it = std::find(packet->data.begin(), packet->data.end(), TERM_CHAR);
        set_status(RRActionStatusE::ACTION_STATE_PROCESSING);
        if (term_it != packet->data.end()) {
            buffer_.append(packet->data.begin(), term_it);
            buf_complete_ = true;
        }
        else {
            buffer_.append(packet->data.begin(), packet->data.end());
            return;
        }

        SerialResponse res;
        if (!res.ParseFromString(buffer_)) {
            RCLCPP_ERROR(logger_, "Failed to deserialize response packet");
            set_status(RRActionStatusE::ACTION_STATE_FAIL);
            return;
        }
        set_status(RRActionStatusE::ACTION_STATE_PROCESSING);
        if (res.op() != op_code_) {
            // ignore response
            set_status(RRActionStatusE::ACTION_STATE_PREPARING);
            return;
        }
        set_res(res);
    }
} // namespace rr_common_plugins