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

/**
 * Provides a dummy GPIO plugin that does not interact with actual hardware. This plugin can be used for testing
 * and development purposes when hardware access is not available or desired.
 */

#pragma once
#include "rr_common_base/rr_gpio_plugin_iface.hpp"
#include "rr_common_plugins/visibility_control.h"
#include <memory>
#include <pluginlib/class_list_macros.hpp>

namespace rr_common_plugins
{
class RrGpioDummyPlugin : public rrobots::interfaces::RRGPIOInterface
{
public:
  CallbackReturn configure(const rclcpp_lifecycle::State& state,
                           rclcpp_lifecycle::LifecycleNode::SharedPtr node) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  std::map<std::string, ValueType> hardware_report() const override;

  int initialise() override;

  int terminate() override;

  int set_pin_mode(unsigned pin, int mode) override;

  int set_pull_up_down(unsigned pin, unsigned pud) override;

  uint32_t tick(void) override;

  int digitial_write(unsigned gpio, unsigned level) override;

  int set_isr_func_ex(unsigned gpio, unsigned edge, int timeout, gpio_isr_func_ex_t func, void* userdata) override;
  
  int clear_isr_func(unsigned gpio) override;

  int gpio_hardware_pwm(unsigned pin, unsigned pwm_freq, unsigned pwm_duty_cycle) override;
};
}  // namespace rr_common_plugins