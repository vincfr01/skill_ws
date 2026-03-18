import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from robot_interfaces.srv import CanExecuteTask, StartSkill
from robot_interfaces.action import ExecuteAtomicSkill


class TaskOrchestrator(Node):

    def __init__(self):
        super().__init__('task_orchestrator')

        # Service Eingang
        self.task_srv = self.create_service(
            StartSkill,
            'start_skill',
            self.start_task_cb
        )

        # Ontologie Client
        self.onto_client = self.create_client(
            CanExecuteTask,
            'can_execute_task'
        )

        # Atomic Skill Clients
        self.move_client = ActionClient(self, ExecuteAtomicSkill, 'move')
        self.grip_client = ActionClient(self, ExecuteAtomicSkill, 'grip')
        self.release_client = ActionClient(self, ExecuteAtomicSkill, 'release')

        self.get_logger().info("Task Orchestrator started.")

    # =========================================================
    # SERVICE CALLBACK
    # =========================================================

    def start_task_cb(self, request, response):

        self.get_logger().info(
            f"Incoming Task: robot={request.robot_id}, "
            f"type={request.task_type}"
        )

        if not self.onto_client.wait_for_service(timeout_sec=2.0):
            response.accepted = False
            response.message = "Ontology service not available"
            return response

        onto_req = CanExecuteTask.Request()
        onto_req.robot_id = request.robot_id
        onto_req.task_type = request.task_type

        future = self.onto_client.call_async(onto_req)

        def ontology_done(fut):
            result = fut.result()
            if result is None or not result.can_execute:
                self.get_logger().warn("Ontology rejected task.")
                return

            self.get_logger().info("Ontology approved task.")

            task = request.task_type.lower()

            if task == "move":
                self._run_sequence([
                    ("move", request.target_frame)
                ])

            elif task == "grip":
                self._run_sequence([
                    ("grip", request.object_id)
                ])

            elif task == "release":
                self._run_sequence([
                    ("release", request.object_id)
                ])

            elif task == "pick":
                self._run_sequence([
                    ("release", request.object_id),
                    ("move", "Down1"),
                    ("grip", request.object_id),
                    ("move", "Up1"),
                ])

            elif task == "place":
                self._run_sequence([
                    ("move", "Down1"),
                    ("release", request.object_id),
                    ("move", "Up1"),
                ])

            elif task in ["pick_and_place", "pickandplace"]:
                self._run_sequence([
                    ("move", request.pick_frame),
                    ("release", request.object_id),
                    ("move", "Down1"),
                    ("grip", request.object_id),
                    ("move", "Up1"),
                    ("move", request.place_frame),
                    ("move", "Down1"),
                    ("release", request.object_id),
                    ("move", "Up1"),
                ])

            else:
                self.get_logger().error(f"Unknown task type: {task}")

        future.add_done_callback(ontology_done)

        response.accepted = True
        response.message = "Task accepted. Checking ontology..."
        return response

    # =========================================================
    # SEQUENTIAL EXECUTION ENGINE
    # =========================================================

    def _run_sequence(self, steps):

        self.get_logger().info(f"Starting sequence with {len(steps)} steps.")

        def run_step(index):
            if index >= len(steps):
                self.get_logger().info("Sequence completed successfully.")
                return

            skill_name, target = steps[index]

            client = self._get_client(skill_name)

            if client is None:
                self.get_logger().error(f"Unknown atomic skill: {skill_name}")
                return

            if not client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(f"Action server '{skill_name}' not available")
                return

            self.get_logger().info(f"Step {index+1}/{len(steps)}: {skill_name}({target})")

            goal = ExecuteAtomicSkill.Goal()
            goal.target_id = target
            goal.frame_id = "world"

            send_future = client.send_goal_async(goal)

            def goal_response_callback(fut):
                goal_handle = fut.result()

                if goal_handle is None or not goal_handle.accepted:
                    self.get_logger().error(f"{skill_name} goal rejected")
                    return

                result_future = goal_handle.get_result_async()

                def result_callback(res_fut):
                    result = res_fut.result().result

                    if not result.success:
                        self.get_logger().error(
                            f"{skill_name} failed: {result.message}"
                        )
                        return

                    run_step(index + 1)

                result_future.add_done_callback(result_callback)

            send_future.add_done_callback(goal_response_callback)

        run_step(0)

    # =========================================================
    # HELPER
    # =========================================================

    def _get_client(self, skill_name):
        if skill_name == "move":
            return self.move_client
        elif skill_name == "grip":
            return self.grip_client
        elif skill_name == "release":
            return self.release_client
        else:
            return None


def main():
    rclpy.init()
    node = TaskOrchestrator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()