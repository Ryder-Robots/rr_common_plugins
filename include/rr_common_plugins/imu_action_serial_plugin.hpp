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

#include "rr_common_base/rr_imu_action_plugin_iface.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_plugins/visibility_control.h"
#include "rr_interfaces/action/monitor_imu_action.hpp"
#include <filesystem>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sys/stat.h>
#include "rr_common_plugins/generated/rr_serial.pb.h"
#include <mutex>
#include <deque>


namespace rr_common_plugins
{
    namespace rr_serial_plugins
    {
        /**
         * @class ImuActionSerialPlugin
         * @brief adds all functionality of serial interface to IMU using arduino BLE.
         *
         * Low level communication to Arduino BLE-33 Sense is specific to USB, using transport_drivers ComposableNodeContainer
         * this will be different for other hardware specific implementations such as PX4 for auto-pilot. To compensate for this
         * plugins are used to hide plumbing. The action node will not change in implementation, but the plugin it uses will.
         *
         * The remainder of documentation will focus specifically on hardware implementation.
         */
        class ImuActionSerialPlugin : public rrobots::interfaces::RRImuActionPluginIface
        {
            using ActionType = rr_interfaces::action::MonitorImuAction;
            using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;
            using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
            using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;
            using State = rclcpp_lifecycle::State;

          private:
            // current transaction UUID
            rclcpp_action::GoalUUID uuid_;
            rclcpp_action::GoalResponse goal_response_;
            std::shared_ptr<const typename ActionType::Goal> goal_;
            std::shared_ptr<GoalHandle> goal_handle_ = nullptr;
            std::deque<std::thread> thread_queue_;

            rclcpp::Subscription<UInt8MultiArray>::SharedPtr subscription_ = nullptr;
            rclcpp_lifecycle::LifecyclePublisher<UInt8MultiArray>::SharedPtr publisher_ = nullptr;


            // internal methods, note these are the methods that will do the work.
            void subscriber_cb(const UInt8MultiArray::UniquePtr &packet);
            void execute(const std::shared_ptr<GoalHandle> goal_handle);

            const std::string WRITE_TOPIC_ = "/serial_write";
            const std::string READ_TOPIC_ = "/serial_read";
            const size_t MAX_THREAD = 10;

            // standard mutex to help with threading.
            std::thread execution_thread_;
            std::mutex g_i_mutex_;
            bool is_executing_ = false;
          
          public:
            ImuActionSerialPlugin() : execution_thread_() {}
            ~ImuActionSerialPlugin();

            /**
             * @fn on_configure
             * @brief called by action concrete implementation during configure phase of its lifecycle.
             * 
             * Creates subscriptions to topics "/serial_read" and "/serial_write", if topics are not available, then 
             * action will be considered "failed". This will return CallbackReturn::TRANSITION_CALLBACK_FAILURE.
             * 
             * 
             * @param state previous or current state of concrete node, or the state of previous lifecycle method.
             * @param node concrete node shared pointer, used to create topic subscriptions
             * @return CallbackReturn, this is described in detail in function description.
             */
            [[nodiscard]] CallbackReturn on_configure(const rclcpp_lifecycle::State &state,
                rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;

            /**
             * @fn handle_goal
             * @brief returns availability of the requested goal.
             * 
             * Stores UUID and Goal reference for later use.
             * 
             * @param uuid unique identified provided by ROS2 middleware, note that this retained as part of the return message and should be 
             * used in bagging services, along with stamp to trace results.
             * @param goal raw goal, will be assessed in this method to ensure that it has all required information for action, if it does then ACCEPT_AND_EXECUTE
             * or ACCEPT_AND_DEFER will be returned under the conditions described in detail in the function description.
             * @return GoalResponse, this is described in detail in function description.
             */
            [[nodiscard]] rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const typename ActionType::Goal> goal) override;

            /**
             * @fn handle_cancel
             * @brief cancellation may occur when lifecycle node decides that timeout period has been reached, after node returns
             * ACCEPT_AND_DEFER, and USB has not come alive. In this case current action will be cancelled, and all consequent 
             * actions requests will return REJECT.
             * @brief goal_handle, pointer to goal information.
             * @return Always return ACCEPT
             */
            [[nodiscard]] rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandle> goal_handle) override;
            /**
             * @fn handle_accepted
             * @brief when handle_goal returns ACCEPT_AND_EXECUTE, then this method will be executed in its own thread.
             * 
             * Submits IMU monitor request to /serial_write, feedback will then be immediately set to 'send', it will remain in this 
             * state until Arduino returns serial response on /serial_read that corresponds IMU response. After this, status will be set
             * to 'processing' and method will map protobuf response to ROS2 message format for IMU,  note that calculations for quadratics
             * will be performed on the micro-processor itself, and not as part of this routine.
             * 
             * After mapping a final result will be set, and action should be considered complete.
             */
            void handle_accepted(
                const std::shared_ptr<GoalHandle> goal_handle) override;


            /**
             * @fn on_activate
             * @brief performs lifecycle activation procedure.
             */
            [[nodiscard]] CallbackReturn on_activate(const State &state) override;

            /**
             * @fn on_deactivate
             * @brief performs lifecycle deactivation routines
             */
            [[nodiscard]] CallbackReturn on_deactivate(const State &state) override;
        };
    } // namespace rr_serial_plugins
} // namespace rr_common_plugins