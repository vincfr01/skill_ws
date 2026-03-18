#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "gripper_close_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = node->get_logger();

  // Parameter, damit du "hand" / "panda_hand" setzen kannst
  if (!node->has_parameter("gripper_group")) {
  node->declare_parameter<std::string>("gripper_group", "hand");
  }
  const std::string group = node->get_parameter("gripper_group").as_string();

  // Executor + Spin-Thread (wichtig für joint_states/TF/callbacks)
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  auto model = loader.getModel();
  if (!model) {
    RCLCPP_ERROR(logger, "RobotModel could not be loaded. Is robot_description set?");
  } else {
    auto groups = model->getJointModelGroupNames();
    RCLCPP_INFO(logger, "Available planning groups:");
    for (const auto& g : groups) {
      RCLCPP_INFO(logger, "  - %s", g.c_str());
    }
  }

  // MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface gripper(node, group);

  // kurz warten, bis aktuelle Zustände da sind
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  gripper.setStartStateToCurrentState();

  // Named target "close" muss in der SRDF der Gripper-Gruppe existieren
  gripper.setNamedTarget("close");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool ok = static_cast<bool>(gripper.plan(plan));

  if (!ok) {
    RCLCPP_ERROR(logger, "Gripper planning failed (group='%s', target='close')", group.c_str());
  } else {
    auto res = gripper.execute(plan);
    if (res != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Gripper execute failed (group='%s')", group.c_str());
    } else {
      RCLCPP_INFO(logger, "Gripper closed (group='%s')", group.c_str());
    }
  }

  exec.cancel();
  spinner.join();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}