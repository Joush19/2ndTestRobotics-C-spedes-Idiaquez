

#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

void move_to_pose(
  moveit::planning_interface::MoveGroupInterface &move_group,
  const geometry_msgs::msg::Pose &target_pose,
  rclcpp::Logger logger)
{
  move_group.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    RCLCPP_INFO(logger, "Plan successful, executing...");
    move_group.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
}

int main(int argc, char * argv[])
{
  // Inicializar ROS 2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "menu_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto logger = rclcpp::get_logger("menu_moveit");

  // Crear interfaz de movimiento
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Definir las dos posiciones
  geometry_msgs::msg::Pose pose1;
  pose1.orientation.w = 1.0;
  pose1.position.x = -0.58;
  pose1.position.y = 0.2;
  pose1.position.z = 0.5;

  geometry_msgs::msg::Pose pose2;
  pose2.orientation.w = 0.0;
  pose2.position.x = 0.1;
  pose2.position.y = 0.1;
  pose2.position.z = 0.7;

  // Menú interactivo
  int choice;
  do {
    std::cout << "\n=== Menú de Movimiento del Brazo ===\n";
    std::cout << "1. Mover a Posición 1\n";
    std::cout << "2. Mover a Posición 2\n";
    std::cout << "3. Salir\n";
    std::cout << "Seleccione una opción: ";
    std::cin >> choice;

    switch (choice) {
      case 1:
        RCLCPP_INFO(logger, "Moviendo a posición 1...");
        move_to_pose(move_group, pose1, logger);
        break;
      case 2:
        RCLCPP_INFO(logger, "Moviendo a posición 2...");
        move_to_pose(move_group, pose2, logger);
        break;
      case 3:
        RCLCPP_INFO(logger, "Saliendo del programa...");
        break;
      default:
        std::cout << "Opción no válida. Intente de nuevo.\n";
    }

  } while (choice != 3);

  // Finalizar ROS
  rclcpp::shutdown();
  return 0;
}
