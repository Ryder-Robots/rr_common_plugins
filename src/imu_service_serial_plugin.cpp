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

namespace rr_common_plugins
{
    namespace rr_serial_plugins
    {

        CallbackReturn ImuServiceSerialPlugin::on_srv_configure(const State &state, LifecycleNode::SharedPtr node)
        {
            return CallbackReturn::SUCCESS;
        }

        GoalResponse ImuServiceSerialPlugin::handle_goal(
            const GoalUUID &uuid,
            std::shared_ptr<const typename ImuServiceSerialPlugin::ActionType::Goal> goal)
        {
            return GoalResponse::ACCEPT_AND_EXECUTE;
        }

        CancelResponse ImuServiceSerialPlugin::handle_cancel(
            const std::shared_ptr<ImuServiceSerialPlugin::GoalHandle> goal_handle)
        {
            return CancelResponse::ACCEPT;
        }

        void ImuServiceSerialPlugin::handle_accepted(
            const std::shared_ptr<ImuServiceSerialPlugin::GoalHandle> goal_handle)
        {
            // implementation
        }

    } // namespace rclcpp_action
} // namespace rr_common_plugins