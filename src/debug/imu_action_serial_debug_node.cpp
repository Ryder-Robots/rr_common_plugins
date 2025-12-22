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


#include "rclcpp_action/rclcpp_action.hpp"
#include "rr_common_plugins/imu_action_serial_plugin.hpp"
#include "rr_common_base/rr_imu_action_plugin_iface.hpp"
#include <memory>
#include <rclcpp_lifecycle/state.hpp>

using namespace rr_common_plugins::rr_serial_plugins;

class RRSerialPluginsDebug : public rclcpp_lifecycle::LifecycleNode
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using State = rclcpp_lifecycle::State;
    using RRImuActionPluginIface = rrobots::interfaces::RRImuActionPluginIface;
    using MonitorImuAction = rr_interfaces::action::MonitorImuAction;


  public:
    explicit RRSerialPluginsDebug(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp_lifecycle::LifecycleNode("rr_imu_action_debug_node", options)
    {
        plugin_ = std::make_unique<ImuActionSerialPlugin>();
    }

    ~RRSerialPluginsDebug() = default;

    CallbackReturn on_configure(const State &state) override
    {
        return plugin_->on_configure(state, this->shared_from_this());
    }

    CallbackReturn on_activate(const State &state) override
    {
        auto handle_goal = std::bind(&RRImuActionPluginIface::handle_goal, plugin_.get(), std::placeholders::_1, std::placeholders::_2);
        auto handle_cancel = std::bind(&RRImuActionPluginIface::handle_cancel, plugin_.get(), std::placeholders::_1);
        auto handle_accepted = std::bind(&RRImuActionPluginIface::handle_accepted, plugin_.get(), std::placeholders::_1);

        action_server_ = rclcpp_action::create_server<MonitorImuAction>(
            this,
            "imu_monitor_action",
            handle_goal,
            handle_cancel,
            handle_accepted);
        return plugin_->on_activate(state);
    }

    CallbackReturn on_deactivate(const State &state) override
    {
        return plugin_->on_deactivate(state);
    }

    CallbackReturn on_cleanup(const State &state) override
    {
        return plugin_->on_cleanup(state);
    }


  private:
    std::unique_ptr<ImuActionSerialPlugin> plugin_;
    rclcpp_action::Server<MonitorImuAction>::SharedPtr action_server_;
};

/**
 * used for debugging purposes. Allows breakpoints to placed within the 
 * imu_action_serial_plugin.
 * 
 * Note that after debugging is completed, then these nodes should be deleted.
 */
int main(int argc, char *argv[])
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    rclcpp::init(argc, argv);

    auto node_lf = std::make_shared<RRSerialPluginsDebug>();
    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node_lf->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}