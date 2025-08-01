#include "include/ros2_control_demo_example_2/diffbot_system.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

//#include "MiniPID.h"

namespace ros2_control_demo_example_2
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init( 
  const hardware_interface::HardwareInfo & info) 
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"]; 
  cfg_.enc_l_counts_per_rev = std::stoi(info_.hardware_parameters["enc_l_counts_per_rev"]);
  cfg_.enc_r_counts_per_rev = std::stoi(info_.hardware_parameters["enc_r_counts_per_rev"]);
  
  wheel_l.h = lgGpiochipOpen(4);
  wheel_r.h = lgGpiochipOpen(4);
  wheel_l.setup(cfg_.left_wheel_name, cfg_.enc_l_counts_per_rev, 0); 
  wheel_r.setup(cfg_.right_wheel_name, 0, cfg_.enc_r_counts_per_rev);

  //MiniPID testing- 
  //pid_l = MiniPID(0.4, 0, 0);
  //pid_r = MiniPID(0.4, 0, 0);
  pid_l.setOutputLimits(-255,255);
  pid_r.setOutputLimits(-255,255);


  param_node_ = rclcpp::Node::make_shared("diffbot_hardware_params");
  param_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  param_executor_->add_node(param_node_);

  param_thread_ = std::thread([this](){
  param_executor_->spin(); 
  });

  param_node_->declare_parameter("pid_left.kp", rclcpp::ParameterValue(0.8));
  param_node_->declare_parameter("pid_left.ki", rclcpp::ParameterValue(0.0));
  param_node_->declare_parameter("pid_left.kd", rclcpp::ParameterValue(0.0));

  param_node_->declare_parameter("pid_right.kp", rclcpp::ParameterValue(0.8));
  param_node_->declare_parameter("pid_right.ki", rclcpp::ParameterValue(0.0));
  param_node_->declare_parameter("pid_right.kd", rclcpp::ParameterValue(0.0));

  double left_kp = param_node_->get_parameter("pid_left.kp").as_double();
  double left_ki = param_node_->get_parameter("pid_left.ki").as_double();
  double left_kd = param_node_->get_parameter("pid_left.kd").as_double();

  double right_kp = param_node_->get_parameter("pid_right.kp").as_double();
  double right_ki = param_node_->get_parameter("pid_right.ki").as_double();
  double right_kd = param_node_->get_parameter("pid_right.kd").as_double();

  pid_left = PIDController(left_kp, left_ki, left_kd);
  pid_right = PIDController(right_kp, right_ki, right_kd);

  param_callback_handle_ = param_node_->add_on_set_parameters_callback
  ( std::bind(&DiffBotSystemHardware::onParameterChange, this, std::placeholders::_1));

  RCLCPP_INFO(*logger_, "PID parameters initialized - Left: P=%.3f I=%.3f D=%.3f, Right: P=%.3f I=%.3f D=%.3f", 
    left_kp, left_ki, left_kd, right_kp, right_ki, right_kd);

  //pid_left = PIDController(0.8,0,0);
  //pid_right = PIDController(0.8,0,0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(), 
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
 
    state_interfaces.emplace_back(hardware_interface::StateInterface(  
        wheel_l.name, hardware_interface::HW_IF_POSITION, &wheel_l.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l.name, hardware_interface::HW_IF_VELOCITY, &wheel_l.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r.name, hardware_interface::HW_IF_POSITION, &wheel_r.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r.name, hardware_interface::HW_IF_VELOCITY, &wheel_r.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
   
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_l.name, hardware_interface::HW_IF_VELOCITY, &wheel_l.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        wheel_r.name, hardware_interface::HW_IF_VELOCITY, &wheel_r.cmd));
  

  return command_interfaces;
}

//ACTIVATION
hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  wheel_l.setMotorPins(25,12); 
  wheel_r.setMotorPins(16,13); 
  wheel_l.setEncPins(17,27);
  wheel_r.setEncPins(6,5);
  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

//DEACTIVATION
hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  wheel_l.stop();
  wheel_r.stop();

  if(param_executor_)
  {
    param_executor_->cancel();
  }
  if(param_thread_.joinable())
  {
    param_thread_.join();
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

//READ
hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  //read encoder values

  double delta_seconds = period.seconds();
  int current_count_l = wheel_l.en_count;
  int count_diff_l = current_count_l - wheel_l.prev_en_count;
  wheel_l.vel = (count_diff_l * wheel_l.rads_l_per_count) / delta_seconds; //vel in rad/s
  wheel_l.prev_en_count = current_count_l;
  wheel_l.pos = wheel_l.getLeftEncPos();
  
  int current_count_r = wheel_r.en_count;
  int count_diff_r = current_count_r - wheel_r.prev_en_count;
  wheel_r.vel = (count_diff_r * wheel_r.rads_r_per_count) / delta_seconds;
  wheel_r.prev_en_count = current_count_r;
  wheel_r.pos = wheel_r.getRightEncPos();
  
  //RCLCPP_INFO(get_logger(), "Left en = %d & %f & Right en_ct = %d & %f ", wheel_l.en_count, wheel_l.vel, wheel_r.en_count, wheel_r.vel); 
  return hardware_interface::return_type::OK;
}

//WRITE
hardware_interface::return_type ros2_control_demo_example_2 ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();    
  //RCLCPP_INFO(get_logger(), "left : %f and right : %f", wheel_l.cmd, wheel_r.cmd);
   
  //PWM CONTROL -
  //double pid_effort_left = pid_left.compute(wheel_l.cmd, wheel_l.vel, dt);
  //double pid_effort_right = pid_right.compute(wheel_r.cmd, wheel_r.vel, dt);
  //wheel_l.setMotorEffort(pid_effort_left);
  //wheel_r.setMotorEffort(pid_effort_right);
  
  //SPEED IN RAD/S CONTROL - 
  // double left_speed = pid_left.compute(wheel_l.cmd, wheel_l.vel, dt); 
  // double right_speed = pid_right.compute(wheel_r.cmd, wheel_r.vel, dt);
  // std::cout<<"speedl-"<<left_speed<<"   speedr-"<<right_speed<<std::endl;
  // wheel_l.setMotorSpeed(left_speed);
  // wheel_r.setMotorSpeed(right_speed);
  


  // Using MiniPID
  // double output_l = pid_l.getOutput(wheel_l.vel, wheel_l.cmd);
  // double output_r = pid_r.getOutput(wheel_r.vel, wheel_r.cmd);
  // std::cout<<"outputl-"<<output_l<<"outputr-"<<output_r<<std::endl;
  // wheel_l.setMotorPWM(output_l);
  // wheel_r.setMotorPWM(output_r);

  
 
  
  //DIRECT SPEED CONTROL - NO PID
  wheel_l.setMotorSpeed(wheel_l.cmd);
  wheel_r.setMotorSpeed(wheel_r.cmd);
  
  RCLCPP_INFO(get_logger(), "LEFT count = %d current = %f & setpoint = %f ||| RIGHT count = %d current = %f & setpoint = %f",wheel_l.en_count, wheel_l.vel, wheel_l.cmd, wheel_r.en_count, wheel_r.vel, wheel_r.cmd);
  
  return hardware_interface::return_type::OK;
}

rcl_interfaces::msg::SetParametersResult DiffBotSystemHardware::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto & param : parameters) 
  {
    double value;

    if(param.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      value = static_cast<double>(param.as_int());
    }
    else
    {
      value = param.as_double();
    }

    if (param.get_name() == "pid_left.kp") {
      pid_left.setKp(value);
      RCLCPP_INFO(*logger_, "Updated left PID Kp to: %.3f", value);
    }
    else if (param.get_name() == "pid_left.ki") {
      pid_left.setKi(value);
      RCLCPP_INFO(*logger_, "Updated left PID Ki to: %.3f", value);
    }
    else if (param.get_name() == "pid_left.kd") {
      pid_left.setKd(value);
      RCLCPP_INFO(*logger_, "Updated left PID Kd to: %.3f", value);
    }
    else if (param.get_name() == "pid_right.kp") {
      pid_right.setKp(value);
      RCLCPP_INFO(*logger_, "Updated right PID Kp to: %.3f", value);
    }
    else if (param.get_name() == "pid_right.ki") {
      pid_right.setKi(value);
      RCLCPP_INFO(*logger_, "Updated right PID Ki to: %.3f", value);
    }
    else if (param.get_name() == "pid_right.kd") {
      pid_right.setKd(value);
      RCLCPP_INFO(*logger_, "Updated right PID Kd to: %.3f", value);
    }
  }
  
  return result;
}


}  // namespace ros2_control_demo_example_2


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)