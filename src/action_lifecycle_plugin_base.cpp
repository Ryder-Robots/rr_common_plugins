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
        buf_complete(RRActionStatusE::ACTION_STATE_PREPARING);
        publisher_->on_activate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RRActionPluginBase::on_deactivate(const State &state)
    {
        (void)state;
        const std::lock_guard<std::mutex> lock(*mutex_);
        buf_complete_no_lock(RRActionStatusE::ACTION_STATE_PREPARING);
        publisher_->on_deactivate();
        deactivate_ = true;

        // sleep for 500 ms to allow for callback to complete.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        subscription_ = nullptr;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RRActionPluginBase::on_cleanup(const State &state)
    {
        (void)state;
        return CallbackReturn::SUCCESS;
    }

    void RRActionPluginBase::buf_complete_no_lock(RRActionStatusE status)
    {
        RCLCPP_DEBUG(logger_, "cleared buffer");
        status_ = status;
        buf_complete_ = true;
        buffer_.clear();
    }

    void RRActionPluginBase::buf_complete(RRActionStatusE status)
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        buf_complete_no_lock(status);
    }

    void RRActionPluginBase::set_status(RRActionStatusE status)
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        status_ = status;
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
        if (!req.SerializeToString(&serialized_data)) {
            RCLCPP_ERROR(logger_, "Failed to serialize request");
            buf_complete(RRActionStatusE::ACTION_STATE_FAIL);
            return false;
        }
        serialized_data.push_back(TERM_CHAR);
        UInt8MultiArray msg;
        msg.data.resize(serialized_data.size());
        std::memcpy(msg.data.data(), serialized_data.data(), serialized_data.size());
        const std::lock_guard<std::mutex> lock(*mutex_);
        buf_complete_no_lock(RRActionStatusE::ACTION_STATE_PREPARING);
        publisher_->publish(msg);
        return true;
    }

    size_t RRActionPluginBase::buffer_size()
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        return buffer_.size();
    }

    bool RRActionPluginBase::is_buf_complete()
    {
        const std::lock_guard<std::mutex> lock(*mutex_);
        return buf_complete_;
    }

    void RRActionPluginBase::subscriber_cb(const UInt8MultiArray::UniquePtr &packet)
    {
        // return immediately on deactivate.
        {
            const std::lock_guard<std::mutex> lock(*mutex_);
            if (deactivate_) {
                return;
            }
        }
        RCLCPP_DEBUG(logger_, "subscriber called");
        if (is_buf_complete()) {
            const std::lock_guard<std::mutex> lock(*mutex_);
            RCLCPP_DEBUG(logger_, "buffer was completed, prepareing for new request");
            status_ = RRActionStatusE::ACTION_STATE_PREPARING;
            buffer_.clear();
            buf_complete_ = false;
        }

        // Do not reset packet, could be just an empty packet
        if (!packet || packet->data.empty()) {
            RCLCPP_DEBUG(logger_, "recieved empty packet call");
            return;
        }

        if (buffer_size() + packet->data.size() > BUFSIZ) {
            RCLCPP_ERROR(logger_, "packet too large");
            buf_complete(RRActionStatusE::ACTION_STATE_FAIL);
            return;
        }

        auto term_it = std::find(packet->data.begin(), packet->data.end(), TERM_CHAR);
        set_status(RRActionStatusE::ACTION_STATE_PROCESSING);
        if (term_it != packet->data.end()) {
            const std::lock_guard<std::mutex> lock(*mutex_);
            buffer_.append(packet->data.begin(), term_it);
            buf_complete_ = true;
            RCLCPP_DEBUG(logger_, "buffer size = '%zu'", buffer_.size());
        }
        else {
            const std::lock_guard<std::mutex> lock(*mutex_);
            buffer_.append(packet->data.begin(), packet->data.end());
            RCLCPP_DEBUG(logger_, "buffer size = '%zu'", buffer_.size());
            return;
        }

        SerialResponse res;
        {
            const std::lock_guard<std::mutex> lock(*mutex_);
            if (!res.ParseFromString(buffer_)) {
                RCLCPP_ERROR(logger_, "Failed to deserialize response packet");
                buf_complete_no_lock(RRActionStatusE::ACTION_STATE_FAIL);
                res_avail_ = true;
                return;
            }
        }
        set_status(RRActionStatusE::ACTION_STATE_PROCESSING);
        if (res.op() != op_code_) {
            // ignore response
            RCLCPP_DEBUG(logger_, "Ignoring response with mismatched op_code: expected %d, got %d",
                static_cast<int>(op_code_), res.op());

            // make sure that in a preparing state, not processing.
            buf_complete(RRActionStatusE::ACTION_STATE_PREPARING);
            return;
        }
        buf_complete(RRActionStatusE::ACTION_STATE_SUCCESS);
        set_res(res);
    }
} // namespace rr_common_plugins