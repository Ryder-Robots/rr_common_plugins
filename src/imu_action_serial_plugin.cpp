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

#include "rr_common_plugins/imu_action_serial_plugin.hpp"


namespace rr_common_plugins
{
    namespace rr_serial_plugins
    {

        using State = rclcpp_lifecycle::State;
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
        using GoalResponse = rclcpp_action::GoalResponse;
        using GoalUUID = rclcpp_action::GoalUUID;
        using ActionType = rr_interfaces::action::MonitorImuAction;
        using CancelResponse = rclcpp_action::CancelResponse;

        void ImuActionSerialPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
        {
            
        }

        GoalResponse ImuActionSerialPlugin::handle_goal(const GoalUUID &uuid, std::shared_ptr<const typename ActionType::Goal> goal)
        {
            (void)uuid;
            (void)goal;
            {
                const std::lock_guard<std::mutex> lock(*mutex_);
                if (is_executing_) {
                    RCLCPP_WARN(logger_, "resource is busy with last request, rejecting new request.");
                    return GoalResponse::REJECT;
                }
                is_executing_ = true;
                is_cancelling_ = false;
            }

            // thread should be joinable at this point.
            if (execution_thread_.joinable()) {
                execution_thread_.join();
            }
            return GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // set cancel to true, this should trigger thread to be joinable.
        CancelResponse ImuActionSerialPlugin::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
        {
            const std::lock_guard<std::mutex> lock(*mutex_);

            if (!goal_handle || is_cancelling_ || !is_executing_) {
                RCLCPP_WARN(logger_, "thread completed or already cancelled");
                return CancelResponse::REJECT;
            }

            if (goal_handle->is_executing()) {
                is_cancelling_ = true;
            }
            return CancelResponse::ACCEPT;
        }

        void ImuActionSerialPlugin::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
        {
            if (execution_thread_.joinable()) {
                execution_thread_.join();
            }
            {
                const std::lock_guard<std::mutex> lock(*mutex_);
                is_cancelling_ = false;
                is_executing_ = true;

                // move assignment for safety.
                execution_thread_ = std::thread();
                // publish request then this needs to be done be done in thread.
                auto execute_in_thread = [this, goal_handle]() { return this->execute(goal_handle); };
                execution_thread_ = std::thread(execute_in_thread);
            }
        }

        CallbackReturn ImuActionSerialPlugin::on_configure(const State &state, LifecycleNode::SharedPtr node)
        {
            return action_plugin_base_.on_configure(state, node);
        }

        CallbackReturn ImuActionSerialPlugin::on_activate(const State &state)
        {
            return action_plugin_base_.on_activate(state);
        }

        CallbackReturn ImuActionSerialPlugin::on_deactivate(const State &state)
        {
            return action_plugin_base_.on_deactivate(state);
        }

        CallbackReturn ImuActionSerialPlugin::on_cleanup(const State &state)
        {
            return action_plugin_base_.on_cleanup(state);
        }

    } // namespace rr_serial_plugins
} // namespace rr_common_plugins
PLUGINLIB_EXPORT_CLASS(rr_common_plugins::rr_serial_plugins::ImuActionSerialPlugin, rrobots::interfaces::RRImuActionPluginIface)


// using rr_common_plugins::rr_serial_plugins::ImuActionSerialPlugin;
// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
// using State = rclcpp_lifecycle::State;
// using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
// using GoalUUID = rclcpp_action::GoalUUID;
// using GoalResponse = rclcpp_action::GoalResponse;
// using CancelResponse = rclcpp_action::CancelResponse;
// using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;
// using IMUFeedback = rr_interfaces::action::MonitorImuAction_Feedback;
// using RRActionStatusE = rr_constants::rr_action_status_t;
// using RROpCodeE = rr_constants::rr_op_code_t;

// using namespace std::placeholders;

// ImuActionSerialPlugin::~ImuActionSerialPlugin()
// {
//     // This should not be necessary, it is here in case of critical malfunction, thread shoudl be
//     // closed at deactivate.
//     if (execution_thread_.joinable()) {
//         execution_thread_.join();
//     }
// }

// /**
//  * called each time a packet is received.
//  */
// void ImuActionSerialPlugin::subscriber_cb(const UInt8MultiArray::UniquePtr &packet)
// {
//     if (!packet || packet->data.empty()) {
//         return;
//     }

//     // Quick state check with minimal lock
//     bool should_process = false;
//     {
//         const std::lock_guard<std::mutex> lock(g_i_mutex_);
//         if (is_executing_ && goal_handle_ && goal_handle_->is_executing()) {
//             should_process = true;
//         }
//     }

//     if (!should_process) {
//         return;
//     }

//     // Check buffer overflow before accumulating
//     if (buffer_.size() + packet->data.size() > BUFSIZ) {
//         RCLCPP_ERROR(logger_, "Buffer overflow - packet too large");
//         buffer_.clear();
//         abort_goal_with_error(RRActionStatusE::ACTION_STATE_FAIL);
//         return;
//     }

//     // Efficiently accumulate data and check for terminator
//     bool complete = false;
//     auto term_it = std::find(packet->data.begin(), packet->data.end(), TERM_CHAR);

//     if (term_it != packet->data.end()) {
//         buffer_.append(packet->data.begin(), term_it);
//         complete = true;
//     } else {
//         buffer_.append(packet->data.begin(), packet->data.end());
//     }

//     if (!complete) {
//         return;  // Wait for more data
//     }

//     // Parse complete message
//     org::ryderrobots::ros2::serial::Response res;
//     if (!res.ParseFromString(buffer_)) {
//         RCLCPP_ERROR(logger_, "Failed to deserialize response packet");
//         buffer_.clear();
//         abort_goal_with_error(RRActionStatusE::ACTION_STATE_FAIL);
//         return;
//     }
//     buffer_.clear();

//     // Filter non-IMU messages
//     if (res.op() != RROpCodeE::MSP_RAW_IMU) {
//         return;
//     }

//     // Build IMU message (no lock needed)
//     sensor_msgs::msg::Imu imu = build_imu_message_from_data(res.msp_raw_imu());

//     // Publish result with lock
//     {
//         const std::lock_guard<std::mutex> lock(g_i_mutex_);

//         // Double-check goal is still valid
//         if (!goal_handle_ || !goal_handle_->is_executing()) {
//             return;
//         }

//         auto result = std::make_shared<ActionType::Result>();
//         result->imu = imu;
//         goal_handle_->succeed(result);

//         auto feedback_msg = std::make_shared<IMUFeedback>();
//         feedback_msg->status = RRActionStatusE::ACTION_STATE_SUCCESS;
//         goal_handle_->publish_feedback(feedback_msg);

//         is_executing_ = false;

//         // Signal completion to unblock execute() timeout wait
//         try {
//             response_promise_.set_value();
//         }
//         catch (const std::future_error &) {
//             // Promise already satisfied or moved, ignore
//         }
//     }
// }

// /*
//  * Plugin uses threads to allow immediate returns for lifecycle calls, however only
//  * one request is processed at a given time.
//  */
// void ImuActionSerialPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
// {
//     auto result = std::make_shared<ActionType::Result>();
//     {
//         const std::lock_guard<std::mutex> lock(g_i_mutex_);
//         is_executing_ = true;
//         goal_handle_ = goal_handle;
//         response_promise_ = std::promise<void>();
//         response_future_ = response_promise_.get_future();
//     }

//     std::shared_ptr<IMUFeedback> feedback_msg = std::make_shared<IMUFeedback>();
//     feedback_msg->status = RRActionStatusE::ACTION_STATE_PREPARING;
//     goal_handle->publish_feedback(feedback_msg);

//     org::ryderrobots::ros2::serial::Request req;
//     org::ryderrobots::ros2::serial::Monitor *monitor = req.mutable_monitor();
//     monitor->set_is_request(true);
//     req.set_op(RROpCodeE::MSP_RAW_IMU);

//     std::string serialized_data;

//     if (!req.SerializeToString(&serialized_data)) {
//         RCLCPP_ERROR(logger_, "Failed to serialize request");
//         feedback_msg->status = RRActionStatusE::ACTION_STATE_FAIL;
//         goal_handle->publish_feedback(feedback_msg);
//         result->success = false;
//         goal_handle->abort(result);

//         // Just clean up and return - no waiting!
//         {
//             const std::lock_guard<std::mutex> lock(g_i_mutex_);
//             is_executing_ = false;
//             is_cancelling_ = false;
//         }
//         return;
//     }

//     // Create UInt8MultiArray
//     std_msgs::msg::UInt8MultiArray msg;
//     serialized_data.push_back(TERM_CHAR);
//     msg.data.resize(serialized_data.size());
//     std::memcpy(msg.data.data(), serialized_data.data(), serialized_data.size());

//     // Publish
//     publisher_->publish(msg);
//     feedback_msg->status = RRActionStatusE::ACTION_STATE_SENT;
//     goal_handle->publish_feedback(feedback_msg);

//     // NOW wait for response (only once, only after publishing)
//     //DEBUG code auto timeout = std::chrono::seconds(5);
//     auto timeout = std::chrono::seconds(500);
//     auto status = response_future_.wait_for(timeout);
//     {
//         const std::lock_guard<std::mutex> lock(g_i_mutex_);

//         // Handle cancellation
//         if (is_cancelling_) {
//             try {
//                 response_future_.get();
//             }
//             catch (const std::exception &e) {
//                 RCLCPP_INFO(logger_, "Goal canceled: %s", e.what());
//                 goal_handle->canceled(result);
//             }
//         }
//         else if (status == std::future_status::timeout && goal_handle->is_executing()) {
//             RCLCPP_ERROR(logger_, "Timeout waiting for IMU response");
//             feedback_msg->status = RRActionStatusE::ACTION_STATE_FAIL;
//             goal_handle->publish_feedback(feedback_msg);
//             result->success = false;
//             goal_handle->abort(result);
//         }
//         // If status == ready and !is_cancelling_, success handled in subscriber_cb

//         is_executing_ = false;
//         is_cancelling_ = false;
//     }
// }

// CallbackReturn ImuActionSerialPlugin::on_configure(const State &state, LifecycleNode::SharedPtr node)
// {
//     (void)state;
//     auto topic_names_and_types = node->get_topic_names_and_types();
//     auto found = false;
//     for (std::string t : {"/serial_read", "/serial_write"}) {
//         for (const auto &[topic_name, type_list] : topic_names_and_types) {
//             if (topic_name == t) {
//                 found = true;
//                 break;
//             }
//         }
//         if (!found) {
//             RCLCPP_ERROR(logger_, "topics are not available");
//             return CallbackReturn::FAILURE;
//         }
//         found = false;
//     }

//     // // create subscriptions
//     auto plugin_cb_ = std::bind(&ImuActionSerialPlugin::subscriber_cb, this, _1);
//     rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
//     publisher_ = node->create_publisher<UInt8MultiArray>(WRITE_TOPIC_, rclcpp::SensorDataQoS(), options);
//     subscription_ = node->create_subscription<UInt8MultiArray>(READ_TOPIC_, rclcpp::SensorDataQoS(), plugin_cb_);

//     return CallbackReturn::SUCCESS;
// }

// GoalResponse ImuActionSerialPlugin::handle_goal(const GoalUUID &uuid, std::shared_ptr<const typename ActionType::Goal> goal)
// {
//     (void)uuid;
//     (void)goal;
//     {
//         const std::lock_guard<std::mutex> lock(g_i_mutex_);

//         // reset is_executing.
//         if (goal_handle_ && !goal_handle_->is_executing()) {
//             is_executing_ = false;
//         }

//         if (is_executing_) {
//             // Resource busy.
//             RCLCPP_WARN(logger_, "resource is busy with last request, rejecting new request.");
//             return GoalResponse::REJECT;
//         }
//         if (execution_thread_.joinable()) {
//             execution_thread_.join();
//         }
//     }
//     return GoalResponse::ACCEPT_AND_EXECUTE;
// }

// CancelResponse ImuActionSerialPlugin::handle_cancel(
//     const std::shared_ptr<GoalHandle> goal_handle)
// {
//     const std::lock_guard<std::mutex> lock(g_i_mutex_);

//     // Verify this is the current goal
//     if (goal_handle_ != goal_handle) {
//         RCLCPP_WARN(logger_, "Cancel request for non-current goal");
//         return CancelResponse::REJECT;
//     }

//     if (!is_executing_) {
//         RCLCPP_WARN(logger_, "Cannot cancel - goal not executing");
//         return CancelResponse::REJECT;
//     }

//     // Set canceling flag
//     is_cancelling_ = true;

//     // Wake up execute() thread immediately by satisfying the promise
//     try {
//         response_promise_.set_exception(
//             std::make_exception_ptr(std::runtime_error("Goal canceled by user")));
//     }
//     catch (const std::future_error &) {
//         // Promise already satisfied, that's fine
//     }

//     RCLCPP_INFO(logger_, "Goal cancellation requested");
//     return CancelResponse::ACCEPT;
// }

// /**
//  * while this routine should not block, it also should not allow more than one thread to run.
//  * therefore there is only one future that is accepted.
//  */
// void ImuActionSerialPlugin::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
// {
//     std::lock_guard<std::mutex> lock(g_i_mutex_);
//     if (execution_thread_.joinable()) {
//         execution_thread_.join();
//     }


// }

// CallbackReturn ImuActionSerialPlugin::on_activate(const State &state)
// {
//     (void)state;
//     publisher_->on_activate();
//     return CallbackReturn::SUCCESS;
// }

// CallbackReturn ImuActionSerialPlugin::on_deactivate(const State &state)
// {
//     (void)state;

//     // gracefully wait for last request
//     if (execution_thread_.joinable()) {
//         execution_thread_.join();
//     }

//     publisher_->on_deactivate();
//     return CallbackReturn::SUCCESS;
// }

// CallbackReturn ImuActionSerialPlugin::on_cleanup(const State &state)
// {
//     (void)state;
//     // Ensure thread is stopped
//     if (execution_thread_.joinable()) {
//         execution_thread_.join();
//     }

//     std::lock_guard<std::mutex> lock(g_i_mutex_);
//     // Reset resources
//     subscription_.reset();
//     publisher_.reset();
//     goal_handle_ = nullptr;
//     is_executing_ = false;
//     is_cancelling_ = false;

//     return CallbackReturn::SUCCESS;
// }

// void ImuActionSerialPlugin::abort_goal_with_error(RRActionStatusE status)
// {
//     const std::lock_guard<std::mutex> lock(g_i_mutex_);

//     if (!goal_handle_ || !is_executing_) {
//         return;
//     }

//     auto result = std::make_shared<ActionType::Result>();
//     result->success = false;

//     auto feedback = std::make_shared<IMUFeedback>();
//     feedback->status = status;

//     goal_handle_->publish_feedback(feedback);
//     goal_handle_->abort(result);
//     is_executing_ = false;
// }

// sensor_msgs::msg::Imu ImuActionSerialPlugin::build_imu_message_from_data(
//     const org::ryderrobots::ros2::serial::MspRawImu& imu_data)
// {
//     rclcpp::Clock clock(RCL_ROS_TIME);
//     sensor_msgs::msg::Imu imu;
//     imu.header.frame_id = rr_constants::LINK_IMU;
//     imu.header.stamp = clock.now();

//     if (imu_data.has_orientation()) {
//         imu.orientation.x = imu_data.orientation().x();
//         imu.orientation.y = imu_data.orientation().y();
//         imu.orientation.z = imu_data.orientation().z();
//         imu.orientation.w = imu_data.orientation().w();
//         for (int i = 0; i < imu_data.orientation_covariance_size(); i++) {
//             imu.orientation_covariance[i] = imu_data.orientation_covariance(i);
//         }
//     }

//     if (imu_data.has_angular_velocity()) {
//         imu.angular_velocity.x = imu_data.angular_velocity().x();
//         imu.angular_velocity.y = imu_data.angular_velocity().y();
//         imu.angular_velocity.z = imu_data.angular_velocity().z();
//         for (int i = 0; i < imu_data.angular_velocity_covariance_size(); i++) {
//             imu.angular_velocity_covariance[i] = imu_data.angular_velocity_covariance(i);
//         }
//     }

//     if (imu_data.has_linear_acceleration()) {
//         imu.linear_acceleration.x = imu_data.linear_acceleration().x();
//         imu.linear_acceleration.y = imu_data.linear_acceleration().y();
//         imu.linear_acceleration.z = imu_data.linear_acceleration().z();
//         for (int i = 0; i < imu_data.linear_acceleration_covariance_size(); i++) {
//             imu.linear_acceleration_covariance[i] = imu_data.linear_acceleration_covariance(i);
//         }
//     }

//     return imu;
// }