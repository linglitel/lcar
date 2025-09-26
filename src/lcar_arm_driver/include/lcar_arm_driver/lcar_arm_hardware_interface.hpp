// 文件路径: lcar_arm_driver/include/lcar_arm_driver/lcar_arm_hardware_interface.hpp

#ifndef LCAR_ARM_HARDWARE_INTERFACE_HPP_
#define LCAR_ARM_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace lcar_arm_driver
{
class LcarArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LcarArmHardwareInterface);

  // ros2_control生命周期函数的标准重写
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // 声明这个硬件能提供和接收哪些接口
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 核心的读写函数
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 我们只需要向量来存储状态和指令
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
};

}  // namespace lcar_arm_driver

#endif  // LCAR_ARM_HARDWARE_INTERFACE_HPP_