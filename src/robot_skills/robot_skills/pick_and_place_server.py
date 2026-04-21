import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient

from robot_interfaces.action import ExecuteAtomicSkill, StartSkill


class PickAndPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_and_place_server')

        self._action_server = ActionServer(
            self, StartSkill, 'pick_and_place', self.execute_callback
        )

        self.pick_client = ActionClient(self, ExecuteAtomicSkill, 'pick')
        self.move_client = ActionClient(self, ExecuteAtomicSkill, 'move')
        self.place_client = ActionClient(self, ExecuteAtomicSkill, 'place')

    async def _call_action(self, client: ActionClient, goal: ExecuteAtomicSkill.Goal, step: str):
        while not client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"[{step}] Waiting for action server...")

        send_future = client.send_goal_async(goal)
        gh = await send_future

        if not gh.accepted:
            return False, f"[{step}] goal rejected"

        result_future = gh.get_result_async()
        res = await result_future

        if not res.result.success:
            return False, f"[{step}] failed: {res.result.message}"

        return True, f"[{step}] ok"

    async def execute_callback(self, goal_handle):
        """
        Ablauf:
          move(pick_frame) -> pick(object_id) -> move(place_frame) -> place(object_id)
        """
        try:
            obj = goal_handle.request.object_id
            pick_frame = goal_handle.request.pick_frame
            place_frame = goal_handle.request.place_frame
            frame_id = goal_handle.request.frame_id or "world"

            self.get_logger().info(
                f"PickAndPlace: object={obj}, pick_frame={pick_frame}, place_frame={place_frame}, frame={frame_id}"
            )

            feedback = StartSkill.Feedback()

            # 1) Move to pick_frame
            feedback.current_step = f"Move to {pick_frame}"
            goal_handle.publish_feedback(feedback)

            g = ExecuteAtomicSkill.Goal()
            g.target_id = pick_frame
            g.frame_id = frame_id
            ok, msg = await self._call_action(self.move_client, g, "MoveToPick")
            if not ok:
                goal_handle.abort()
                res = StartSkill.Result()
                res.success = False
                res.message = msg
                return res

            # 2) Pick
            feedback.current_step = f"Pick {obj}"
            goal_handle.publish_feedback(feedback)

            g = ExecuteAtomicSkill.Goal()
            g.target_id = obj
            g.frame_id = frame_id
            ok, msg = await self._call_action(self.pick_client, g, "Pick")
            if not ok:
                goal_handle.abort()
                res = StartSkill.Result()
                res.success = False
                res.message = msg
                return res

            # 3) Move to place_frame
            feedback.current_step = f"Move to {place_frame}"
            goal_handle.publish_feedback(feedback)

            g = ExecuteAtomicSkill.Goal()
            g.target_id = place_frame
            g.frame_id = frame_id
            ok, msg = await self._call_action(self.move_client, g, "MoveToPlace")
            if not ok:
                goal_handle.abort()
                res = StartSkill.Result()
                res.success = False
                res.message = msg
                return res

            # 4) Place
            feedback.current_step = f"Place {obj}"
            goal_handle.publish_feedback(feedback)

            g = ExecuteAtomicSkill.Goal()
            g.target_id = obj
            g.frame_id = frame_id
            ok, msg = await self._call_action(self.place_client, g, "Place")
            if not ok:
                goal_handle.abort()
                res = StartSkill.Result()
                res.success = False
                res.message = msg
                return res

            goal_handle.succeed()
            res = StartSkill.Result()
            res.success = True
            res.message = f"PickAndPlace finished: move({pick_frame}) -> pick -> move({place_frame}) -> place"
            return res

        except Exception as e:
            self.get_logger().error(f"Exception in PickAndPlaceServer: {e}")
            goal_handle.abort()
            res = StartSkill.Result()
            res.success = False
            res.message = f"Exception: {e}"
            return res


def main():
    rclpy.init()
    node = PickAndPlaceServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()