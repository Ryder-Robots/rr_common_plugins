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

#include "rr_common_plugins/imu_service_serial_plugin.hpp"

using rr_common_plugins::rr_serial_plugins::ImuServiceSerialPlugin;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;
using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using GoalUUID = rclcpp_action::GoalUUID;
using GoalResponse = rclcpp_action::GoalResponse;
using CancelResponse = rclcpp_action::CancelResponse;
using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;
using IMUFeedback = rr_interfaces::action::MonitorImuAction_Feedback;
using RRActionStatusE = rr_constants::rr_action_status_t;
using RROpCodeE = rr_constants::rr_op_code_t;

using namespace std::placeholders;

namespace rr_common_plugins
{
    namespace rr_serial_plugins
    {

        /**
         * called each time a packet is recieved.
         */
        void ImuServiceSerialPlugin::subscriber_cb(const UInt8MultiArray::UniquePtr &packet)
        {
            const std::lock_guard<std::mutex> lock(g_i_mutex_);
            if (goal_handle_ == nullptr) {
                RCLCPP_FATAL(rclcpp::get_logger("ImuServiceSerialPlugin"),
                    "goal handle was not created");
                return;
            }
            auto result = std::make_shared<ActionType::Result>();
            std::shared_ptr<IMUFeedback> feedback_msg = std::make_shared<IMUFeedback>();
            feedback_msg->status = RRActionStatusE::ACTION_STATE_PROCESSING;
            goal_handle_->publish_feedback(feedback_msg);

            // if not executing leave method.
            if (!goal_handle_->is_executing()) {
                RCLCPP_ERROR(rclcpp::get_logger("ImuServiceSerialPlugin"),
                    "Not able to process request at this time");
                feedback_msg->status = RRActionStatusE::ACTION_STATE_FAIL;
                goal_handle_->publish_feedback(feedback_msg);
                result->success = false;
                goal_handle_->abort(result);
                return;
            }

            // begin deserializing packet.
            org::ryderrobots::ros2::serial::Response res;
            if (!res.ParseFromArray(packet->data.data(), packet->data.size())) {
                RCLCPP_ERROR(rclcpp::get_logger("ImuServiceSerialPlugin"),
                    "Failed to deserialize response packet");
                feedback_msg->status = RRActionStatusE::ACTION_STATE_FAIL;
                goal_handle_->publish_feedback(feedback_msg);
                result->success = false;
                goal_handle_->abort(result);
                return;
            }

            if (res.op() != RROpCodeE::MSP_RAW_IMU) {
                // not an error, just dont need to care about anything that is not IMU.
                return;
            }
            const auto &imu_data = res.msp_raw_imu();
            rclcpp::Clock clock(RCL_ROS_TIME);
            sensor_msgs::msg::Imu imu;
            imu.header.frame_id = rr_constants::LINK_IMU;
            imu.header.stamp = clock.now();

            if (imu_data.has_orientation()) {
                imu.orientation.x = imu_data.orientation().x();
                imu.orientation.y = imu_data.orientation().y();
                imu.orientation.z = imu_data.orientation().z();
                imu.orientation.w = imu_data.orientation().w();
                for (auto i = 0; i < imu_data.orientation_covariance_size(); i++) {
                    imu.orientation_covariance[i] = imu_data.orientation_covariance(i);
                }
            }

            if (imu_data.has_angular_velocity()) {
                imu.angular_velocity.x = imu_data.angular_velocity().x();
                imu.angular_velocity.y = imu_data.angular_velocity().y();
                imu.angular_velocity.z = imu_data.angular_velocity().z();
                for (auto i = 0; i < imu_data.angular_velocity_covariance_size(); i++) {
                    imu.angular_velocity_covariance[i] = imu_data.angular_velocity_covariance(i);
                }
            }

            if (imu_data.has_linear_acceleration()) {
                imu.linear_acceleration.x = imu_data.linear_acceleration().x();
                imu.linear_acceleration.y = imu_data.linear_acceleration().y();
                imu.linear_acceleration.z = imu_data.linear_acceleration().z();
                for (auto i = 0; i < imu_data.linear_acceleration_covariance_size(); i++) {
                    imu.linear_acceleration_covariance[i] = imu_data.linear_acceleration_covariance(i);
                }
            }

            if (rclcpp::ok()) {
                result->imu = imu;
                goal_handle_->succeed(result);
                feedback_msg->status = RRActionStatusE::ACTION_STATE_SUCCESS;
                goal_handle_->publish_feedback(feedback_msg);
            }
        }

        void ImuServiceSerialPlugin::execute(const std::shared_ptr<GoalHandle> goal_handle)
        {
            auto result = std::make_shared<ActionType::Result>();
            {
                const std::lock_guard<std::mutex> lock(g_i_mutex_);
                goal_handle_ = goal_handle;
            }

            std::shared_ptr<IMUFeedback> feedback_msg = std::make_shared<IMUFeedback>();
            feedback_msg->status = RRActionStatusE::ACTION_STATE_PREPARING;
            goal_handle->publish_feedback(feedback_msg);

            org::ryderrobots::ros2::serial::Request req;
            org::ryderrobots::ros2::serial::Monitor *monitor = req.mutable_monitor();
            monitor->set_is_request(true);
            req.set_op(RROpCodeE::MSP_RAW_IMU);

            std::string serialized_data;

            if (!req.SerializeToString(&serialized_data)) {
                RCLCPP_ERROR(rclcpp::get_logger("ImuServiceSerialPlugin"),
                    "Failed to serialize request");

                feedback_msg->status = RRActionStatusE::ACTION_STATE_FAIL;
                goal_handle->publish_feedback(feedback_msg);
                result->success = false;
                goal_handle->abort(result);
                return;
            }

            // Create UInt8MultiArray
            std_msgs::msg::UInt8MultiArray msg;
            msg.data.resize(serialized_data.size());
            std::memcpy(msg.data.data(), serialized_data.data(), serialized_data.size());

            // Publish
            publisher_->publish(msg);
            feedback_msg->status = RRActionStatusE::ACTION_STATE_SENT;
            goal_handle->publish_feedback(feedback_msg);
        }

        /**
         * Verify that the file references character device.
         */
        bool ImuServiceSerialPlugin::is_character_device(const std::string &path)
        {
            struct stat buffer;
            if (stat(path.c_str(), &buffer) != 0) {
                return false;
            }
            return S_ISCHR(buffer.st_mode);
        }


        /**
         * TODO: When other plugins are added move checking to common base class.
         * Perform various checks to ensure transport node is available.
         */
        uint8_t ImuServiceSerialPlugin::transport_available(LifecycleNode::SharedPtr node)
        {
            auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
                node,
                "serial_driver");

            if (!parameters_client->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(node->get_logger(), "serial_driver parameter service unavailable");
                return 1;
            }

            auto future = parameters_client->get_parameters({"device_name"});

            if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(1)) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "Failed to retrieve device_name parameter");
                return 2;
            }

            auto values = future.get();
            if (values.empty()) {
                RCLCPP_ERROR(node->get_logger(), "device_name parameter not found");
                return 3;
            }

            std::string device_name = values[0].as_string();
            if (device_name.empty()) {
                RCLCPP_ERROR(node->get_logger(), "device_name parameter is empty");
                return 4;
            }

            // protect with mutex
            {
                const std::lock_guard<std::mutex> lock(g_i_mutex_);
                device_name_ = device_name;
            }

            if (!std::filesystem::exists(device_name)) {
                RCLCPP_WARN(node->get_logger(), "Device does not exist: %s", device_name.c_str());
                return 5;
            }

            if (!is_character_device(device_name)) {
                RCLCPP_ERROR(node->get_logger(), "Device is not a character device: %s", device_name.c_str());
                return 6;
            }

            RCLCPP_INFO(node->get_logger(), "Transport available: %s", device_name.c_str());
            return 0;
        }

        // TODO: move to on_configure, mkae it clearer.
        CallbackReturn ImuServiceSerialPlugin::on_srv_configure(const State &state, LifecycleNode::SharedPtr node)
        {
            (void)state;

            switch (transport_available(node)) {
            case 0:
                RCLCPP_DEBUG(node->get_logger(), "Node checks passed inilizing");
                break;

            case 1:
            case 2:
            case 3:
                RCLCPP_ERROR(node->get_logger(), "possible recoverable error, can be attempted later");
                return CallbackReturn::FAILURE;

            case 4:
            case 5:
            case 6:
                RCLCPP_FATAL(node->get_logger(), "unrecoverable error");
                return CallbackReturn::ERROR;
            }


            auto topic_names_and_types = node->get_topic_names_and_types();
            auto found = false;
            for (std::string t : {"/serial_read", "/serial_write"}) {
                for (const auto &[topic_name, type_list] : topic_names_and_types) {
                    if (topic_name == t) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    RCLCPP_ERROR(node->get_logger(), "topics are not available");
                    return CallbackReturn::FAILURE;
                }
                found = false;
            }

            // // create subscriptions
            auto plugin_cb_ = std::bind(&ImuServiceSerialPlugin::subscriber_cb, this, _1);
            rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
            publisher_ = node->create_publisher<UInt8MultiArray>(WRITE_TOPIC_, rclcpp::SensorDataQoS(), options);
            subscription_ = node->create_subscription<UInt8MultiArray>(READ_TOPIC_, rclcpp::SensorDataQoS(), plugin_cb_);

            return CallbackReturn::SUCCESS;
        }

        GoalResponse ImuServiceSerialPlugin::handle_goal(const GoalUUID &uuid, std::shared_ptr<const typename ActionType::Goal> goal)
        {
            {
                const std::lock_guard<std::mutex> lock(g_i_mutex_);
                if (!(std::filesystem::exists(device_name_) && is_character_device(device_name_))) {
                    RCLCPP_WARN(rclcpp::get_logger("ImuServiceSerialPlugin"), "Device does not exist: %s", device_name_.c_str());
                    return GoalResponse::ACCEPT_AND_DEFER;
                }
            }
            goal_ = goal;
            uuid_ = uuid;
            return GoalResponse::ACCEPT_AND_EXECUTE;
        }

        CancelResponse ImuServiceSerialPlugin::handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle)
        {
            (void)goal_handle;
            return CancelResponse::ACCEPT;
        }

        void ImuServiceSerialPlugin::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
        {
            // publish request then this needs to be done be done in thread.
            auto execute_in_thread = [this, goal_handle]() { return this->execute(goal_handle); };
            std::thread {execute_in_thread}.detach();
        }

        CallbackReturn ImuServiceSerialPlugin::on_activate(const State &state)
        {
            (void)state;
            publisher_->on_activate();
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn ImuServiceSerialPlugin::on_deactivate(const State &state)
        {
            (void)state;
            publisher_->on_deactivate();
            return CallbackReturn::SUCCESS;
        }

    } // namespace rr_serial_plugins
} // namespace rr_common_plugins