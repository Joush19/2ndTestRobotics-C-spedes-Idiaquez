#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("add_two_boxes_to_rviz2");
  auto logger = rclcpp::get_logger("add_two_boxes_to_rviz2");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //  caja 1
  moveit_msgs::msg::CollisionObject box1;
  box1.header.frame_id = "world";
  box1.id = "box1";

  shape_msgs::msg::SolidPrimitive primitive1;
  primitive1.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive1.dimensions = {0.4, 0.2, 0.2};

  geometry_msgs::msg::Pose pose1;
  pose1.orientation.w = 1.0;
  pose1.position.x = 0.8;
  pose1.position.y = 0.2;
  pose1.position.z = 0.1;

  box1.primitives.push_back(primitive1);
  box1.primitive_poses.push_back(pose1);
  box1.operation = moveit_msgs::msg::CollisionObject::ADD;

  // caja 2
  moveit_msgs::msg::CollisionObject box2;
  box2.header.frame_id = "world";
  box2.id = "box2";

  shape_msgs::msg::SolidPrimitive primitive2;
  primitive2.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive2.dimensions = {0.3, 0.3, 0.3};

  geometry_msgs::msg::Pose pose2;
  pose2.orientation.w = 1.0;
  pose2.position.x = -0.1;
  pose2.position.y = 0.6;
  pose2.position.z = 0.14;

  box2.primitives.push_back(primitive2);
  box2.primitive_poses.push_back(pose2);
  box2.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Aplicar ambos objetos
  planning_scene_interface.applyCollisionObjects({box1, box2});

  RCLCPP_INFO(logger, "Se añadieron las dos cajas a la escena de planificación.");

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
