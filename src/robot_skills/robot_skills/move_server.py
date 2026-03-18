import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.duration import Duration

from robot_interfaces.action import ExecuteAtomicSkill

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState

import tf2_ros


class MoveSkillServer(Node):
    def __init__(self):
        super().__init__('move_skill_server')

        self.base_frame = "world"
        self.ee_frame = "panda_link8"
        self.moveit_group = "panda_arm"
        self.delta_z = 0.10  

        self._action_server = ActionServer(
            self, ExecuteAtomicSkill, 'move', self.execute_callback
        )

        self.traj_client = ActionClient(
            self, FollowJointTrajectory, '/panda_arm_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            "panda_joint1", "panda_joint2", "panda_joint3",
            "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
        ]

        self.named_joint_targets = {
            "Home": [0.0, -0.6, 0.0, -2.0, 0.0, 1.4, 0.8],
            "PoseA": [0.0, 0.4, 0.0, -1.8, 0.0, 2.2, 0.8],
            "PoseB": [1.0, 0.4, 0.0, -1.8, 0.0, 2.2, 0.8],
            "PoseC": [1.4, 0.0, 0.2, -1.8, 0.0, 1.8, 1.0],
            "PoseD": [0.5, 0.0, 0.2, -1.8, 0.0, 2.4, 1.0],            
        }

        self.current_joint_positions = None   
        self.current_joint_state = None      

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')

        self.get_logger().info("MoveSkillServer ready. Targets: "
                               f"{list(self.named_joint_targets.keys()) + ['Up1','Down1']}")


    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

        joint_map = dict(zip(msg.name, msg.position))
        positions = []
        for name in self.joint_names:
            if name not in joint_map:
                return  
            positions.append(joint_map[name])
        self.current_joint_positions = positions

    def get_current_tcp_pose(self) -> PoseStamped:
        """
        Holt aktuelle TCP Pose (Endeffektor) via TF.
        """
        trans = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.ee_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=1.0)
        )

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = trans.transform.translation.x
        pose.pose.position.y = trans.transform.translation.y
        pose.pose.position.z = trans.transform.translation.z
        pose.pose.orientation = trans.transform.rotation
        return pose

    def compute_linear_cartesian_traj(self, target_id: str) -> JointTrajectory:
        """
        Berechnet eine lineare kartesische Bewegung in Z-Richtung (Up1/Down1)
        über MoveIt /compute_cartesian_path und gibt eine JointTrajectory zurück.
        """
        if self.current_joint_state is None:
            raise RuntimeError("No /joint_states received yet.")

        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("MoveIt service /compute_cartesian_path not available.")

        current_pose = self.get_current_tcp_pose()

        dz = self.delta_z if target_id == "Up1" else -self.delta_z
        target_pose = PoseStamped()
        target_pose.header.frame_id = current_pose.header.frame_id
        target_pose.pose = current_pose.pose
        target_pose.pose.position.z += dz


        req = GetCartesianPath.Request()
        req.group_name = self.moveit_group
        req.link_name = self.ee_frame
        req.max_step = 0.01          
        req.jump_threshold = 0.0   
        req.avoid_collisions = True

        start_state = RobotState()
        start_state.joint_state = self.current_joint_state
        req.start_state = start_state

        req.waypoints.append(target_pose.pose)

        future = self.cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            raise RuntimeError("Cartesian path computation failed (no response).")

        res = future.result()

        if res.fraction < 0.99:
            raise RuntimeError(f"Cartesian path incomplete (fraction={res.fraction:.2f}).")

        jt = res.solution.joint_trajectory
        if not jt.points:
            raise RuntimeError("MoveIt returned empty trajectory.")

        return jt

    async def send_joint_trajectory(self, joint_traj: JointTrajectory, time_sec: int = 2):
        """
        Schickt eine JointTrajectory an den FollowJointTrajectory Controller.
        """
        while not self.traj_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for trajectory action server...")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj

        send_future = self.traj_client.send_goal_async(goal)
        traj_goal_handle = await send_future

        if not traj_goal_handle.accepted:
            return False, "Trajectory goal rejected"

        result_future = traj_goal_handle.get_result_async()
        traj_result = await result_future

        if traj_result.result.error_code != 0:
            return False, f"Controller error_code={traj_result.result.error_code}"

        return True, "OK"

    async def execute_callback(self, goal_handle):
        try:
            target_id = goal_handle.request.target_id
            self.get_logger().info(f"Move: target_id={target_id}")

            while self.current_joint_positions is None:
                self.get_logger().info("Waiting for joint states...")
                rclpy.spin_once(self, timeout_sec=0.1)

            if target_id in ["Up1", "Down1"]:
                joint_traj = self.compute_linear_cartesian_traj(target_id)

                ok, msg = await self.send_joint_trajectory(joint_traj)
                if not ok:
                    goal_handle.abort()
                    result = ExecuteAtomicSkill.Result()
                    result.success = False
                    result.message = msg
                    return result

                goal_handle.succeed()
                result = ExecuteAtomicSkill.Result()
                result.success = True
                result.message = f"Moved linearly ({target_id})"
                return result

            if target_id in self.named_joint_targets:
                traj = JointTrajectory()
                traj.joint_names = self.joint_names

                point = JointTrajectoryPoint()
                point.positions = self.named_joint_targets[target_id]
                point.time_from_start.sec = 2
                point.time_from_start.nanosec = 0
                traj.points = [point]

                ok, msg = await self.send_joint_trajectory(traj)
                if not ok:
                    goal_handle.abort()
                    result = ExecuteAtomicSkill.Result()
                    result.success = False
                    result.message = msg
                    return result

                goal_handle.succeed()
                result = ExecuteAtomicSkill.Result()
                result.success = True
                result.message = f"Moved to {target_id}"
                return result

            goal_handle.abort()
            result = ExecuteAtomicSkill.Result()
            result.success = False
            result.message = (
                f"Unknown target_id '{target_id}'. Use: "
                f"{list(self.named_joint_targets.keys()) + ['Up1','Down1']}"
            )
            return result

        except Exception as e:
            self.get_logger().error(f"Exception in MoveSkillServer: {e}")
            goal_handle.abort()
            result = ExecuteAtomicSkill.Result()
            result.success = False
            result.message = f"Exception: {e}"
            return result


def main():
    rclpy.init()
    node = MoveSkillServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()