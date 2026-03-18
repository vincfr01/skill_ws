import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from robot_interfaces.action import ExecuteAtomicSkill


class PickServer(Node):
    def __init__(self):
        super().__init__('pick_server')

        self._action_server = ActionServer(
            self, ExecuteAtomicSkill, 'pick', self.execute_callback
        )

        self.release_client = ActionClient(self, ExecuteAtomicSkill, 'release')
        self.move_client = ActionClient(self, ExecuteAtomicSkill, 'move')
        self.grip_client = ActionClient(self, ExecuteAtomicSkill, 'grip')

    async def _call_atomic(self, client: ActionClient, goal: ExecuteAtomicSkill.Goal, step_name: str):
        """
        Helper: ruft einen Atomic Skill als Action auf und wartet auf Ergebnis.
        """
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"[{step_name}] Waiting for action server...")

        send_future = client.send_goal_async(goal)
        goal_handle = await send_future

        if not goal_handle.accepted:
            return False, f"[{step_name}] goal rejected"

        result_future = goal_handle.get_result_async()
        result = await result_future

        if not result.result.success:
            return False, f"[{step_name}] failed: {result.result.message}"

        return True, f"[{step_name}] ok"

    async def execute_callback(self, goal_handle):
        """
        Pick = Release(workpiece) -> Move(Down1) -> Grip(workpiece) -> Move(Up1)
        """
        try:
            workpiece_id = goal_handle.request.target_id
            frame_id = goal_handle.request.frame_id

            self.get_logger().info(
                f"Pick goal received: workpiece={workpiece_id}, frame={frame_id}"
            )

            release_goal = ExecuteAtomicSkill.Goal()
            release_goal.target_id = workpiece_id
            release_goal.frame_id = frame_id

            ok, msg = await self._call_atomic(self.release_client, release_goal, "Release")
            if not ok:
                goal_handle.abort()
                result = ExecuteAtomicSkill.Result()
                result.success = False
                result.message = msg
                return result

            down_goal = ExecuteAtomicSkill.Goal()
            down_goal.target_id = "Down1"
            down_goal.frame_id = frame_id

            ok, msg = await self._call_atomic(self.move_client, down_goal, "MoveDown")
            if not ok:
                goal_handle.abort()
                result = ExecuteAtomicSkill.Result()
                result.success = False
                result.message = msg
                return result

            grip_goal = ExecuteAtomicSkill.Goal()
            grip_goal.target_id = workpiece_id
            grip_goal.frame_id = frame_id

            ok, msg = await self._call_atomic(self.grip_client, grip_goal, "Grip")
            if not ok:
                goal_handle.abort()
                result = ExecuteAtomicSkill.Result()
                result.success = False
                result.message = msg
                return result

            up_goal = ExecuteAtomicSkill.Goal()
            up_goal.target_id = "Up1"
            up_goal.frame_id = frame_id

            ok, msg = await self._call_atomic(self.move_client, up_goal, "MoveUp")
            if not ok:
                goal_handle.abort()
                result = ExecuteAtomicSkill.Result()
                result.success = False
                result.message = msg
                return result

            goal_handle.succeed()
            result = ExecuteAtomicSkill.Result()
            result.success = True
            result.message = f"Pick finished for {workpiece_id}: Release -> Down1 -> Grip -> Up1"
            return result

        except Exception as e:
            self.get_logger().error(f"Exception in PickServer: {e}")
            goal_handle.abort()
            result = ExecuteAtomicSkill.Result()
            result.success = False
            result.message = f"Exception: {e}"
            return result


def main():
    rclpy.init()
    node = PickServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()