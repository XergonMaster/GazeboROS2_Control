// #ifndef DIFF_BOT_CONTROLLER__DIFF_BOT_CONTROLLER_HPP_
// #define DIFF_BOT_CONTROLLER__DIFF_BOT_CONTROLLER_HPP_
#ifndef DIFF_BOT_CONTROLLER__DIFF_BOT_CONTROLLER_HPP_
#define DIFF_BOT_CONTROLLER__DIFF_BOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace diff_bot_controller
{
  class DiffBotController : public controller_interface::ControllerInterface
  {
  public:
    DiffBotController();

    // Funciones virtuales puras obligatorias
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // Callback para el subscriber de /cmd_vel
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Interfaces de comando para las ruedas izquierda y derecha
    std::vector<hardware_interface::LoanedCommandInterface> left_wheel_command_;
    std::vector<hardware_interface::LoanedCommandInterface> right_wheel_command_;

    // Suscripción al tópico /cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Variables para almacenar las velocidades deseadas
    double left_wheel_velocity_command_;
    double right_wheel_velocity_command_;
  };
} // namespace diff_bot_controller

#endif // DIFF_BOT_CONTROLLER__DIFF_BOT_CONTROLLER_HPP_
