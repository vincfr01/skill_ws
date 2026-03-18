#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "move_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("move_program");

  // === (3a) Executor + Spin-Thread: direkt nach Node-Erstellung ===
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });

  // MoveGroupInterface danach erstellen (nutzt ROS callbacks / subscriptions)
  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

  // Optional: etwas Zeit geben, damit first joint_states/TF ankommen
  rclcpp::sleep_for(std::chrono::milliseconds(200));

  // === (3b) Startzustand setzen: direkt vor dem Planen ===
  move_group.setStartStateToCurrentState();

  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0, 0, -3.14/2);
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

  geometry_msgs::msg::Pose goal_pose;
  goal_pose.orientation = msg_quat;
  goal_pose.position.x = 0.3;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.4;

  move_group.setNamedTarget("close");

  move_group.setPoseTarget(goal_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  auto const success = static_cast<bool>(move_group.plan(plan1));

  if (success)
  {
    move_group.execute(plan1);
  }
  else
  {
    RCLCPP_ERROR(logger, "We were not able to plan and execute!");
  }

  // === (3c) Aufräumen: ganz am Ende vor shutdown ===
  exec.cancel();
  spinner.join();

  rclcpp::shutdown();
  return 0;
}
