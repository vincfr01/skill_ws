import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from robot_interfaces.action import ExecuteAtomicSkill


class PlaceServer(Node):
    def __init__(self):
        super().__init__('place_server')

        self._action_server = ActionServer(
            self, ExecuteAtomicSkill, 'place', self.execute_callback
        )

        self.release_client = ActionClient(self, ExecuteAtomicSkill, 'release')
        self.move_client = ActionClient(self, ExecuteAtomicSkill, 'move')

    async def _call_atomic(self, client: ActionClient, goal: ExecuteAtomicSkill.Goal, step_name: str):
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
        try:
            workpiece_id = goal_handle.request.target_id
            frame_id = goal_handle.request.frame_id

            self.get_logger().info(f"Place goal received: workpiece={workpiece_id}, frame={frame_id}")

            down_goal = ExecuteAtomicSkill.Goal()
            down_goal.target_id = "Down1"
            down_goal.frame_id = frame_id

            ok, msg = await self._call_atomic(self.move_client, down_goal, "MoveDown")
            if not ok:
                goal_handle.abort()
                res = ExecuteAtomicSkill.Result()
                res.success = False
                res.message = msg
                return res

            rel_goal = ExecuteAtomicSkill.Goal()
            rel_goal.target_id = workpiece_id
            rel_goal.frame_id = frame_id

            ok, msg = await self._call_atomic(self.release_client, rel_goal, "Release")
            if not ok:
                goal_handle.abort()
                res = ExecuteAtomicSkill.Result()
                res.success = False
                res.message = msg
                return res

            up_goal = ExecuteAtomicSkill.Goal()
            up_goal.target_id = "Up1"
            up_goal.frame_id = frame_id

            ok, msg = await self._call_atomic(self.move_client, up_goal, "MoveUp")
            if not ok:
                goal_handle.abort()
                res = ExecuteAtomicSkill.Result()
                res.success = False
                res.message = msg
                return res

            goal_handle.succeed()
            res = ExecuteAtomicSkill.Result()
            res.success = True
            res.message = f"Place finished for {workpiece_id}: Down1 -> Release -> Up1"
            return res

        except Exception as e:
            self.get_logger().error(f"Exception in PlaceServer: {e}")
            goal_handle.abort()
            res = ExecuteAtomicSkill.Result()
            res.success = False
            res.message = f"Exception: {e}"
            return res


def main():
    rclpy.init()
    node = PlaceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()