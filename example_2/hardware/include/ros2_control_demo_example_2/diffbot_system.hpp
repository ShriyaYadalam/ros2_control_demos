#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "wheel.hpp"
#include "PidController.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "MiniPID.h"

namespace ros2_control_demo_example_2
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{

struct Config

{
std::string left_wheel_name = "";
std::string right_wheel_name = ""; 
int enc_l_counts_per_rev = 0;
int enc_r_counts_per_rev = 0;
};


  
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  DiffBotSystemHardware():pid_l(18,0,0),pid_r(18,0,0){}

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /**
   * \return clock of the SystemInterface.
   */ 
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  Config cfg_; 
  Wheel wheel_l;
  Wheel wheel_r;
  
  PIDController pid_left;
  PIDController pid_right;

  MiniPID pid_l;
  MiniPID pid_r;

  rclcpp::Node::SharedPtr param_node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>param_executor_;
  std::thread param_thread_;

  rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> & parameters); 
  
};

}  // namespace ros2_control_demo_example_2

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_2__DIFFBOT_SYSTEM_HPP_