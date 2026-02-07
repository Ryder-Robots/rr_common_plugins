#include "rr_common_plugins/rr_gpio_dummy_plugin.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using ValueType = std::variant<int, std::string>;

namespace rr_common_plugins
{

CallbackReturn RrGpioDummyPlugin::configure(const rclcpp_lifecycle::State& state,
                                            rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  (void)state;
  (void)node;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RrGpioDummyPlugin::on_activate(const rclcpp_lifecycle::State& state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RrGpioDummyPlugin::on_deactivate(const rclcpp_lifecycle::State& state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RrGpioDummyPlugin::on_cleanup(const rclcpp_lifecycle::State& state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

std::map<std::string, ValueType> RrGpioDummyPlugin::hardware_report() const
{
  return {};
}

int RrGpioDummyPlugin::initialise()
{
  return 0;
}

int RrGpioDummyPlugin::terminate()
{
  return 0;
}

int RrGpioDummyPlugin::set_pin_mode(unsigned pin, int mode)
{
  (void)pin;
  (void)mode;
  return 0;
}

int RrGpioDummyPlugin::set_pull_up_down(unsigned pin, unsigned pud)
{
  (void)pin;
  (void)pud;
  return 0;
}

uint32_t RrGpioDummyPlugin::tick(void)
{
  return 0;
}

int RrGpioDummyPlugin::set_isr_func_ex(unsigned gpio, unsigned edge, int timeout, gpio_isr_func_ex_t func,
                                       void* userdata)
{
  (void)gpio;
  (void)edge;
  (void)timeout;
  (void)func;
  (void)userdata;
  return 0;
}

int RrGpioDummyPlugin::clear_isr_func(unsigned gpio)
{
  (void)gpio;
  return 0;
}

int RrGpioDummyPlugin::digitial_write(unsigned gpio, unsigned level)
{
  (void)gpio;
  (void)level;
  return 0;
}

int RrGpioDummyPlugin::gpio_hardware_pwm(unsigned pin, unsigned pwm_freq, unsigned pwm_duty_cycle)
{
  (void)pin;
  (void)pwm_freq;
  (void)pwm_duty_cycle;
  return 0;
}

}  // namespace rr_common_plugins

PLUGINLIB_EXPORT_CLASS(rr_common_plugins::RrGpioDummyPlugin, rrobots::interfaces::RRGPIOInterface)