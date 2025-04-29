import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance

TRAJECTORIES = {
    "traj0": [
        {
            "positions": [0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": [-0.195016, -1.70093, 0.902027, -0.944217, -1.52982, -0.195171],
            "velocities": [0, 0, 0, 0, 0, 0],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
}

# The example i am creating this from is a Joint Trejectory Controler Client, 
# I am not sure if that is the same aproach that I wil use here.
class MinimalMover(Node):
    def __init__(self):
        super().__init__('minimal_mover')
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        # Create the action client
        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        # Connect to the action server
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        # Creates action messages from trajectories
        self.goals = {}
        for traj_name in TRAJECTORIES:
            goal = JointTrajectory()
            goal.joint_names = self.joints
            for pt in TRAJECTORIES[traj_name]:
                point = JointTrajectoryPoint()
                point.positions = pt["positions"]
                point.velocities = pt["velocities"]
                point.time_from_start = pt["time_from_start"]
                goal.points.append(point)
            self.goals[traj_name] = goal
        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None

        # Starts the first trajectory, and 
        self.execute_next_trajectory()

    def execute_next_trajectory(self):
        if self.i >= len(self.goals):
            self.get_logger().info("Done with all trajectories")
            raise SystemExit
        traj_name = list(self.goals)[self.i]
        self.i = self.i + 1
        if traj_name:
            self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory {traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]

        goal.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
        goal.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=self.joints[i]) for i in range(6)
        ]

        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            time.sleep(2)
            self.execute_next_trajectory()
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"Done with result: {self.error_code_to_str(result.error_code)}"
                )
            raise RuntimeError("Executing trajectory failed. " + result.error_string)

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod
    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"


def main(args=None):
    rclpy.init(args=args)

    jtc_client = MinimalMover()
    try:
        rclpy.spin(jtc_client)
    except RuntimeError as err:
        jtc_client.get_logger().error(str(err))
    except SystemExit:
        rclpy.logging.get_logger("jtc_client").info("Done")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
