// 文件路径: lcar_arm_driver/src/lcar_arm_hardware_interface.cpp

#include "lcar_arm_driver/lcar_arm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <iomanip> // 用于 std::setprecision

namespace lcar_arm_driver
{

hardware_interface::CallbackReturn LcarArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_states_position_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(rclcpp::get_logger("LcarArmHardwareInterface"), "硬件接口初始化成功 [日志打印测试模式]");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LcarArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> LcarArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn LcarArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LcarArmHardwareInterface"), "配置中... [日志打印测试模式]");
  // 没有真实硬件需要配置，直接返回成功
  RCLCPP_INFO(rclcpp::get_logger("LcarArmHardwareInterface"), "配置成功。");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LcarArmHardwareInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LcarArmHardwareInterface"), "清理中... [日志打印测试模式]");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LcarArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 激活时，将指令设置为当前状态，防止机械臂跳动
  for (size_t i = 0; i < hw_states_position_.size(); ++i) {
      hw_commands_[i] = hw_states_position_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("LcarArmHardwareInterface"), "硬件已激活 [日志打印测试模式]");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LcarArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LcarArmHardwareInterface"), "硬件已停用 [日志打印测试模式]");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type LcarArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 在日志打印模式下，我们模拟一个完美的机器人，它能瞬间到达指令位置。
  // 所以，我们直接将指令值复制给状态值。
  RCLCPP_DEBUG(rclcpp::get_logger("LcarArmHardwareInterface"), "[读取 STM32 状态] 模拟完美的状态反馈。");
  for (size_t i = 0; i < hw_states_position_.size(); ++i) {
    hw_states_position_[i] = hw_commands_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LcarArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 在日志打印模式下，我们将指令字符串格式化并打印到控制台。
  std::stringstream ss;
  ss << "W:";
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
      ss << std::fixed << std::setprecision(4) << hw_commands_[i];
      if (i < hw_commands_.size() - 1) {
          ss << ":";
      }
  }
  ss << "\\n"; // 用 \\n 来清晰地表示字符串中的换行符

  RCLCPP_INFO(rclcpp::get_logger("LcarArmHardwareInterface"), "[写入 STM32 指令] %s", ss.str().c_str());
  
  return hardware_interface::return_type::OK;
}

}  // namespace lcar_arm_driver

// 这是将我们的类注册为ros2_control插件的关键宏
PLUGINLIB_EXPORT_CLASS(
  lcar_arm_driver::LcarArmHardwareInterface,
  hardware_interface::SystemInterface)