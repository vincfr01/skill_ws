import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from robot_interfaces.action import ExecuteAtomicSkill
from control_msgs.action import GripperCommand


class GripSkillServer(Node):
    def __init__(self):
        super().__init__('grip_skill_server')

        self._action_server = ActionServer(
            self, ExecuteAtomicSkill, 'grip', self.execute_callback
        )

        self.gripper_client = ActionClient(
            self, GripperCommand, '/panda_hand_controller/gripper_cmd'
        )

    async def execute_callback(self, goal_handle):
        try:
            self.get_logger().info("Grip: closing gripper")

            while not self.gripper_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("Waiting for gripper action server...")

            goal = GripperCommand.Goal()
            goal.command.position = 0.0      # zu
            goal.command.max_effort = 40.0

            send_future = self.gripper_client.send_goal_async(goal)
            gripper_goal_handle = await send_future

            if not gripper_goal_handle.accepted:
                goal_handle.abort()
                result = ExecuteAtomicSkill.Result()
                result.success = False
                result.message = "Gripper goal rejected"
                return result

            result_future = gripper_goal_handle.get_result_async()
            _ = await result_future

            goal_handle.succeed()
            result = ExecuteAtomicSkill.Result()
            result.success = True
            result.message = "Gripper closed"
            return result

        except Exception as e:
            self.get_logger().error(f"Exception in GripSkillServer: {e}")
            goal_handle.abort()
            result = ExecuteAtomicSkill.Result()
            result.success = False
            result.message = f"Exception: {e}"
            return result


def main():
    rclpy.init()
    node = GripSkillServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()