#include "diff_bot/diff_bot_controller.hpp"

#include <string>
#include <vector>

namespace diff_bot_controller
{
    DiffBotController::DiffBotController() : left_wheel_velocity_command_(0.0), right_wheel_velocity_command_(0.0)
    {
    }

    // Implementación de las funciones faltantes
    controller_interface::InterfaceConfiguration DiffBotController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names = {
            "left_wheel/velocity",
            "right_wheel/velocity"};
        return config;
    }

    controller_interface::InterfaceConfiguration DiffBotController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::NONE;
        return config;
    }

    controller_interface::CallbackReturn DiffBotController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DiffBotController::on_configure(const rclcpp_lifecycle::State &)
    {
        // Inicializa el suscriptor a /cmd_vel
        auto node = get_node();
        cmd_vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&DiffBotController::cmd_vel_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node->get_logger(), "Subscribed to /cmd_vel");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DiffBotController::on_activate(const rclcpp_lifecycle::State &)
    {
        // Asegúrate de que las interfaces de comando estén cargadas
        if (left_wheel_command_.empty() || right_wheel_command_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces loaded for left or right wheel");
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "DiffBotController activated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn DiffBotController::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_node()->get_logger(), "DiffBotController deactivated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type DiffBotController::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Aplica las velocidades calculadas a las ruedas izquierda y derecha
        left_wheel_command_[0].set_value(left_wheel_velocity_command_);
        right_wheel_command_[0].set_value(right_wheel_velocity_command_);

        RCLCPP_INFO(get_node()->get_logger(), "Applied velocity - Left wheel: %.2f, Right wheel: %.2f",
                    left_wheel_velocity_command_, right_wheel_velocity_command_);

        return controller_interface::return_type::OK;
    }

    void DiffBotController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Calcula las velocidades de las ruedas a partir del mensaje Twist
        double wheel_separation = 0.2; // Distancia entre ruedas (ejemplo)
        double wheel_radius = 0.05;    // Radio de las ruedas (ejemplo)

        left_wheel_velocity_command_ = (msg->linear.x - msg->angular.z * wheel_separation / 2.0) / wheel_radius;
        right_wheel_velocity_command_ = (msg->linear.x + msg->angular.z * wheel_separation / 2.0) / wheel_radius;

        RCLCPP_INFO(get_node()->get_logger(), "Received cmd_vel - Linear: %.2f, Angular: %.2f", msg->linear.x, msg->angular.z);
    }
} // namespace diff_bot_controller

#include "pluginlib/class_list_macros.hpp"

// Exportar el plugin como un controlador
PLUGINLIB_EXPORT_CLASS(diff_bot_controller::DiffBotController, controller_interface::ControllerInterface)
