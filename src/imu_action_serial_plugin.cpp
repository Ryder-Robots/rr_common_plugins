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
        using IMUFeedback = rr_interfaces::action::MonitorImuAction_Feedback;
        using SerialRequest = org::ryderrobots::ros2::serial::Request;
        using SerialMonitor = org::ryderrobots::ros2::serial::Monitor;
        using RROpCodeE = rr_constants::rr_op_code_t;
        using MonitorImuAction_Result = rr_interfaces::action::MonitorImuAction_Result;
        using RRActionStatusE = rr_constants::rr_action_status_t;
        using Imu = sensor_msgs::msg::Imu;
        using MspRawImu = org::ryderrobots::ros2::serial::MspRawImu;


        Imu ImuActionSerialPlugin::build_imu_message_from_data(const MspRawImu &imu_data)
        {
            Imu imu;
            imu.header.frame_id = rr_constants::LINK_IMU;
            imu.header.stamp = action_plugin_base_.get_time_stamp();

            if (imu_data.has_orientation()) {
                imu.orientation.x = imu_data.orientation().x();
                imu.orientation.y = imu_data.orientation().y();
                imu.orientation.z = imu_data.orientation().z();
                imu.orientation.w = imu_data.orientation().w();
                for (int i = 0; i < imu_data.orientation_covariance_size(); i++) {
                    imu.orientation_covariance[i] = imu_data.orientation_covariance(i);
                }
            }

            if (imu_data.has_angular_velocity()) {
                imu.angular_velocity.x = imu_data.angular_velocity().x();
                imu.angular_velocity.y = imu_data.angular_velocity().y();
                imu.angular_velocity.z = imu_data.angular_velocity().z();
                for (int i = 0; i < imu_data.angular_velocity_covariance_size(); i++) {
                    imu.angular_velocity_covariance[i] = imu_data.angular_velocity_covariance(i);
                }
            }

            if (imu_data.has_linear_acceleration()) {
                imu.linear_acceleration.x = imu_data.linear_acceleration().x();
                imu.linear_acceleration.y = imu_data.linear_acceleration().y();
                imu.linear_acceleration.z = imu_data.linear_acceleration().z();
                for (int i = 0; i < imu_data.linear_acceleration_covariance_size(); i++) {
                    imu.linear_acceleration_covariance[i] = imu_data.linear_acceleration_covariance(i);
                }
            }

            return imu;
        }

        void ImuActionSerialPlugin::cancel_goal(std::shared_ptr<MonitorImuAction_Result> result_msg,
            const std::shared_ptr<GoalHandle> goal_handle, std::shared_ptr<IMUFeedback> feedback_msg)
        {
            result_msg->success = false;
            result_msg->uuid.uuid = uuid_;
            goal_handle->canceled(result_msg);
            {
                const std::lock_guard<std::mutex> lock(*mutex_);
                is_executing_ = false;
            }
            feedback_msg->status = action_plugin_base_.get_status();
            goal_handle->publish_feedback(feedback_msg);
        }

        //TODO: Need a timeout period
        void ImuActionSerialPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
        {
            std::shared_ptr<IMUFeedback> feedback_msg = std::make_shared<IMUFeedback>();
            feedback_msg->status = action_plugin_base_.get_status();
            feedback_msg->uuid.uuid = uuid_;
            goal_handle->publish_feedback(feedback_msg);
            auto result_msg = std::make_shared<MonitorImuAction_Result>();

            SerialRequest req;
            SerialMonitor *monitor = req.mutable_monitor();
            req.set_op(RROpCodeE::MSP_RAW_IMU);
            monitor->set_is_request(true);

            if (!action_plugin_base_.publish(req)) {
                cancel_goal(result_msg, goal_handle, feedback_msg);
                return;
            }
            action_plugin_base_.set_status(RRActionStatusE::ACTION_STATE_SENT);

            // Create an initial wall timer but disable it initally.
            // Common IMU Hz is 104. Allow 100 Ns for a USB cable that is less than 15cm, with
            // average processing time for a NanoBLE33
            rclcpp::Clock clock(RCL_SYSTEM_TIME); 
            auto wait_period = std::chrono::nanoseconds((1000000000ULL / 100) + 100);
            auto start_period = clock.now();
            std::this_thread::sleep_for(wait_period);
            while (!(action_plugin_base_.is_res_avail() || action_plugin_base_.get_status() == RRActionStatusE::ACTION_STATE_FAIL)) {
                std::this_thread::sleep_for(wait_period);
                rclcpp::Duration delta = clock.now() - start_period;

                if (delta.seconds() > timeout_period_) {
                    RCLCPP_ERROR(logger_, "timeout error");
                    action_plugin_base_.set_status(RRActionStatusE::ACTION_STATE_TIMEOUT);
                    cancel_goal(result_msg, goal_handle, feedback_msg);
                }
            }

            // if resp is recieved and successful, then populate IMU variables.
            if (action_plugin_base_.get_status() == RRActionStatusE::ACTION_STATE_FAIL) {
                RCLCPP_ERROR(logger_, "serde failure");
                cancel_goal(result_msg, goal_handle, feedback_msg);
                return;
            }

            if (!action_plugin_base_.is_res_avail()) {
                RCLCPP_ERROR(logger_, "reported as successful but result not available");
                action_plugin_base_.set_status(RRActionStatusE::ACTION_STATE_FAIL);
                cancel_goal(result_msg, goal_handle, feedback_msg);
                return;
            }

            auto result = action_plugin_base_.get_res();
            if (!result.has_msp_raw_imu()) {
                RCLCPP_ERROR(logger_, "IMU data not set");
                action_plugin_base_.set_status(RRActionStatusE::ACTION_STATE_FAIL);
                cancel_goal(result_msg, goal_handle, feedback_msg);
                return;            
            }

            Imu imu = build_imu_message_from_data(result.msp_raw_imu());
            result_msg->imu = imu;
            result_msg->success = true;
            goal_handle->succeed(result_msg);
            is_executing_ = false;
        }

        GoalResponse ImuActionSerialPlugin::handle_goal(const GoalUUID &uuid, std::shared_ptr<const typename ActionType::Goal> goal)
        {
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
            // set UUID after check, if it gets rejected, then we want the UUID that is getting used
            // here.
            uuid_ = uuid;

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

        // Lifecycle is handled by the lifecycle manager.
        CallbackReturn ImuActionSerialPlugin::on_configure(const State &state, LifecycleNode::SharedPtr node)
        {
            node->declare_parameter("timeout_period", 10);

            timeout_period_ = node->get_parameter("timeout_period").as_int();
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
