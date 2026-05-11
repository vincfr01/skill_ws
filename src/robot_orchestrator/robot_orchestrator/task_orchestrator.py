import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from robot_interfaces.srv import CanExecuteTask
from robot_interfaces.action import ExecuteAtomicSkill, StartSkill


class TaskOrchestrator(Node):

    def __init__(self):
        super().__init__('task_orchestrator')

        self.cb_group = ReentrantCallbackGroup()

        # Action Server
        self.action_server = ActionServer(
            self,
            StartSkill,
            'start_skill',
            self.execute_callback,
            callback_group=self.cb_group
        )

        # Ontologie Client
        self.onto_client = self.create_client(
            CanExecuteTask,
            'can_execute_task',
            callback_group=self.cb_group
        )

        # Atomic Skill Clients
        self.move_client = ActionClient(
            self, ExecuteAtomicSkill, 'move',
            callback_group=self.cb_group
        )
        self.grip_client = ActionClient(
            self, ExecuteAtomicSkill, 'grip',
            callback_group=self.cb_group
        )
        self.release_client = ActionClient(
            self, ExecuteAtomicSkill, 'release',
            callback_group=self.cb_group
        )

        self.get_logger().info("Task Orchestrator Action Server started.")


    def execute_callback(self, goal_handle):
        request = goal_handle.request

        self.get_logger().info(
            f"Incoming Task: robot={request.robot_id}, type={request.task_type}"
        )

        result = StartSkill.Result()

        # 1) Ontologie prüfen

        self._publish_feedback(goal_handle, "Checking ontology")

        if not self.onto_client.wait_for_service(timeout_sec=2.0):
            goal_handle.abort()
            result.success = False
            result.message = "Ontology service not available"
            return result

        onto_req = CanExecuteTask.Request()
        onto_req.robot_id = request.robot_id
        onto_req.task_type = request.task_type

        future = self.onto_client.call_async(onto_req)

        done_event = threading.Event()
        result_container = {"result": None, "exception": None}

        def ontology_done(fut):
            try:
                result_container["result"] = fut.result()
            except Exception as e:
                result_container["exception"] = e
            finally:
                done_event.set()

        future.add_done_callback(ontology_done)

        if not done_event.wait(timeout=5.0):
            goal_handle.abort()
            result.success = False
            result.message = "Ontology query timed out"
            return result

        if result_container["exception"] is not None:
            goal_handle.abort()
            result.success = False
            result.message = f"Ontology query failed: {result_container['exception']}"
            return result

        onto_result = result_container["result"]

        if onto_result is None:
            goal_handle.abort()
            result.success = False
            result.message = "Ontology query returned no result"
            return result

        if not onto_result.can_execute:
            goal_handle.abort()
            result.success = False
            result.message = f"Task not executable: {onto_result.reason}"
            return result

        # 2) Task in Skill-Sequenz übersetzen

        task = request.task_type.lower().replace("_", "")

        if task == "move":
            steps = [
                ("move", request.target_frame)
            ]

        elif task == "grip":
            steps = [
                ("grip", request.object_id)
            ]

        elif task == "release":
            steps = [
                ("release", request.object_id)
            ]

        elif task == "pick":
            steps = [
                ("move", request.pick_frame),
                ("release", request.object_id),
                ("move", "Down1"),
                ("grip", request.object_id),
                ("move", "Up1"),
            ]

        elif task == "place":
            steps = [
                ("move", request.place_frame),
                ("move", "Down1"),
                ("release", request.object_id),
                ("move", "Up1"),
            ]

        elif task in ["pickandplace"]:
            steps = [
                ("move", request.pick_frame),
                ("release", request.object_id),
                ("move", "Down1"),
                ("grip", request.object_id),
                ("move", "Up1"),
                ("move", request.place_frame),
                ("move", "Down1"),
                ("release", request.object_id),
                ("move", "Up1"),
            ]

        else:
            goal_handle.abort()
            result.success = False
            result.message = f"Unknown task type: {request.task_type}"
            return result

        # 3) Sequenz ausführen

        ok, msg = self._run_sequence(steps, goal_handle)

        if ok:
            goal_handle.succeed()
            result.success = True
            result.message = msg
        else:
            goal_handle.abort()
            result.success = False
            result.message = msg

        return result


    def _run_sequence(self, steps, goal_handle):
        """
        Führt Atomic Skills streng sequenziell aus.
        Nächster Skill startet erst, wenn der vorherige erfolgreich fertig ist.
        """

        self.get_logger().info(f"Starting sequence with {len(steps)} steps.")

        for index, (skill_name, target) in enumerate(steps, start=1):

            if goal_handle.is_cancel_requested:
                return False, "Task canceled"

            client = self._get_client(skill_name)
            if client is None:
                return False, f"Unknown skill: {skill_name}"

            if not client.wait_for_server(timeout_sec=2.0):
                return False, f"Action server '{skill_name}' not available"

            step_label = f"{index}/{len(steps)} {skill_name}({target})"
            self.get_logger().info(step_label)

            goal = ExecuteAtomicSkill.Goal()
            goal.target_id = target
            goal.frame_id = "world"

            send_event = threading.Event()
            send_container = {"goal_handle": None, "exception": None}

            def atomic_feedback_cb(feedback_msg):
                state = feedback_msg.feedback.state
                self._publish_feedback(goal_handle, f"{step_label} -> {state}")

            send_future = client.send_goal_async(goal, feedback_callback=atomic_feedback_cb)

            def goal_response_callback(fut):
                try:
                    send_container["goal_handle"] = fut.result()
                except Exception as e:
                    send_container["exception"] = e
                finally:
                    send_event.set()

            send_future.add_done_callback(goal_response_callback)

            if not send_event.wait(timeout=5.0):
                return False, f"{skill_name} goal response timed out"

            if send_container["exception"] is not None:
                return False, f"{skill_name} goal failed: {send_container['exception']}"

            atomic_goal_handle = send_container["goal_handle"]
            if atomic_goal_handle is None or not atomic_goal_handle.accepted:
                return False, f"{skill_name} goal rejected"

            result_future = atomic_goal_handle.get_result_async()

            result_event = threading.Event()
            result_container = {"result": None, "exception": None}

            def result_callback(fut):
                try:
                    result_container["result"] = fut.result().result
                except Exception as e:
                    result_container["exception"] = e
                finally:
                    result_event.set()

            result_future.add_done_callback(result_callback)

            if not result_event.wait(timeout=20.0):
                return False, f"{skill_name} execution timed out"

            if result_container["exception"] is not None:
                return False, f"{skill_name} result failed: {result_container['exception']}"

            atomic_result = result_container["result"]
            if atomic_result is None:
                return False, f"{skill_name} returned no result"

            if not atomic_result.success:
                return False, f"{skill_name} failed: {atomic_result.message}"

        return True, "Task completed successfully"

    def _publish_feedback(self, goal_handle, text: str):
        feedback = StartSkill.Feedback()
        feedback.current_step = text
        goal_handle.publish_feedback(feedback)

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

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()