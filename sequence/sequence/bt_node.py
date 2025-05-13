import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient
import py_trees
import py_trees_ros
import time

from messages.msg import MiRState
from messages.srv import MirAppendMission
from messages.msg import Robotiq2FGripperRobotInput as InputMsg
from messages.msg import Robotiq2FGripperRobotOutput as OutputMsg

import trajectories
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
    
class OpenGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, node) -> None:
        super(OpenGripper, self).__init__(name=name)
        # Initialize the blackboard
        self.blackboard = py_trees.blackboard.Client(name='GripperBlackboard')
        for key in ['/griper/state', '/griper/target', '/griper/position', '/griper/moving']:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        print(self.blackboard)
        self.duration = 15
        self.start_time = None
        self.node = node
        self.gripper_publisher = None

    def initialise(self) -> None:
        # Initialize the gripper state
        self.node.get_logger().info("Opening gripper")
        # Initialize timer
        self.start_time = time.time()
        # Create a publisher for gripper messages
        self.gripper_publisher = self.node.create_publisher(OutputMsg, 'Robotiq2FGripperRobotOutput', 10)
        msg = OutputMsg()
        msg.r_pr = 0
        msg.r_sp = 255
        msg.r_fr = 64
        self.gripper_publisher.publish(msg)
        self.node.get_logger().info("Gripper open command sent")
        

    def update(self) -> py_trees.common.Status:
        # Logic to open the gripper
        elapsed_time = time.time() - self.start_time
        print(self.blackboard)
        # Check if the gripper is open
        if elapsed_time > self.duration:
            # If the gripper is not open after the duration, return failure
            self.node.get_logger().info("Gripper is could not be opened")
            return py_trees.common.Status.FAILURE
        elif self.blackboard.griper.position <= 10 and self.blackboard.griper.moving != 0:
            # If the gripper is open, return success
            self.node.get_logger().info("Gripper is open")
            return py_trees.common.Status.SUCCESS
        else:
            # If the gripper is still moving, return running
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status) -> None:
        pass
    
class CloseGripper(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(CloseGripper, self).__init__(name=name)
        # Initialize the blackboard
        self.blackboard = py_trees.blackboard.Client(name='GripperBlackboard')
        for key in ['/griper/state', '/griper/target', '/griper/position', '/griper/moving']:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        print(self.blackboard)
        self.duration = 15
        self.start_time = None
        self.node = node
        self.gripper_publisher = None

    def initialise(self):
        # Initialize the gripper state
        self.node.get_logger().info("Closing gripper")
        # Initialize timer
        self.start_time = time.time()
        # Create a publisher for gripper messages
        self.gripper_publisher = self.node.create_publisher(OutputMsg, 'Robotiq2FGripperRobotOutput', 10)
        msg = OutputMsg()
        msg.r_pr = 255
        msg.r_sp = 255
        msg.r_fr = 255
        self.gripper_publisher.publish(msg)
        self.node.get_logger().info("Gripper close command sent")

    def update(self) -> py_trees.common.Status:
        # Logic to open the gripper
        elapsed_time = time.time() - self.start_time
        print(self.blackboard)
        # Check if the gripper is closed
        if elapsed_time > self.duration:
            # If the gripper is not closed after the duration, return failure
            self.node.get_logger().info("Gripper is could not be closed")
            return py_trees.common.Status.FAILURE
        elif self.blackboard.griper.position >= 245:
            # If no object is detected in griper, return failure
            self.node.get_logger().info("No object detected in gripper")
            return py_trees.common.Status.FAILURE
        elif self.blackboard.griper.moving != 0:
            # If the gripper is closed, return success
            self.node.get_logger().info("Gripper is closed")
            return py_trees.common.Status.SUCCESS
        else:
            # The gripper is still moving, return running
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        pass

class MiRMission(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, mission):
        super(MiRMission, self).__init__(name=name)
        self.node = node
        self.mir_client = None
        self.mission = mission

    def initialise(self):
        # Initialize the blackboard
        self.blackboard = py_trees.blackboard.Client(name='MiRBlackboard')
        for key in ['/mir/state']:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        print(self.blackboard)
        # Initialize the MiR client
        self.node.get_logger().info("Initializing MiR client")
        self.mir_client = self.node.create_client(MirAppendMission, 'MiRAppendMission')
        # Wait for the service to be available
        while not self.mir_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')
        self.node.get_logger().info("MiR client initialized")
        # Send the mission to the MiR
        self.request = MirAppendMission.Request()
        self.request.mission_id = self.mission
        self.future = self.mir_client.call_async(self.request)
    
    def update(self) -> py_trees.common.Status:
        # Logic to wait for the MiR to complete the mission
        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.node.get_logger().info('Service call failed %r' % (e,))
                return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().info('Result: %s' % response.response)
                if response.success:
                    match self.blackboard.mir.state: # MiR state (0 = none, 1 = starting, 2 = shutting down, 3 = ready, 4 = pause, 5 = executing, 6 = aborted, 7 = completed, 8 = docked, 9 = docking, 10 = emergency stop, 11 = manual control, 12 = error)
                        case 3:
                            # MiR is not executing a mission
                            self.node.get_logger().info("MiR is not executing a mission")
                            return py_trees.common.Status.SUCCESS
                        case 4:
                            # MiR is paused
                            self.node.get_logger().info("MiR is paused")
                            return py_trees.common.Status.RUNNING
                        case 5:
                            # MiR is executing the mission
                            self.node.get_logger().info("MiR is executing the mission")
                            return py_trees.common.Status.RUNNING
                        case 6:
                            # MiR mission is aborted
                            self.node.get_logger().info("MiR mission aborted")
                            return py_trees.common.Status.FAILURE
                        case 7:
                            # Mission is completed, return success
                            self.node.get_logger().info("MiR mission completed")
                            return py_trees.common.Status.SUCCESS
                        case 10:
                            # MiR is in an emergency stopp state
                            self.node.get_logger().info("MiR is in an emergency stop state")
                            return py_trees.common.Status.FAILURE
                        case 12:
                            # MiR is in error state
                            self.node.get_logger().info("MiR is in error state")
                            return py_trees.common.Status.FAILURE
                        case default:
                            # MiR is in an unknown state
                            return py_trees.common.Status.FAILURE
                else:
                    # Mission failed to be sent
                    self.node.get_logger().info("MiR mission failed to be sent")
                    return py_trees.common.Status.FAILURE
        else:
            # The mission is still being sent, return running
            return py_trees.common.Status.RUNNING
            
    def terminate(self, new_status):
        pass

class RobotMove(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, trajectory):
        super(RobotMove, self).__init__(name=name)
        # self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        # self.declare_parameter(
        #     "joints",
        #     [
        #         "shoulder_pan_joint",
        #         "shoulder_lift_joint",
        #         "elbow_joint",
        #         "wrist_1_joint",
        #         "wrist_2_joint",
        #         "wrist_3_joint",
        #     ],
        # )

        # self.controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        # self.joints = self.get_parameter("joints").value

        self.controller_name = "scaled_joint_trajectory_controller/follow_joint_trajectory"
        self.joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.node = node
        self.robot_action = None
        self.trajectory = trajectory
        self.running = False
        self.failure = False
        self.finished = False

    def initialise(self):
        # Initialize the blackboard
        self.blackboard = py_trees.blackboard.Client(name='MiRBlackboard')
        for key in ['/mir/state', '/mir/position/x', '/mir/position/y', '/mir/position/w', '/mir/velocity/linear', '/mir/velocity/angular', '/mir/battery_percentage', '/griper/state', '/griper/target', '/griper/position', '/griper/moving']:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        print(self.blackboard)

        # Initialize the robot client
        self.node.get_logger().info("Initializing robot action client")
        self.robot_action = ActionClient(self.node, FollowJointTrajectory, self.controller_name)
        self.node.get_logger().info(f"Waiting for action server on {self.controller_name}")

        # Wait for the action to be available
        self.robot_action.wait_for_server()
        self.node.get_logger().info("Robot action client initialized")

        # Create a message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        for pt in self.trajectory:
            # Create a trajectory point
            point = JointTrajectoryPoint()
            point.positions = pt["positions"]
            point.velocities = pt["velocities"]
            point.time_from_start = pt["time_from_start"]
            trajectory.points.append(point)

        # Send the goal to the robot
        self.node.get_logger().info("Sending goal to robot")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
        goal.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=self.joints[i]) for i in range(6)
        ]
        send_future = self.robot_action.send_goal_async(goal)
        send_future.add_done_callback(self.goal_response_callback)
        self.node.get_logger().info("Goal sendt to robot")
        self.running = True
        self.finished = False
        self.failure = False


    def goal_response_callback(self, future):
        # Callback for the goal response
        goal_handle = future.result()
        if goal_handle.accepted:
            self.node.get_logger().debug("Goal accepted :)")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.get_result_callback)
        else:
            self.node.get_logger().error("Goal rejected :(")
            self.failure = True

        
    def get_result_callback(self, future):
        # Callback for the result
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.finished = True
            self.running = False
        else:
            self.failure = True

    def update(self) -> py_trees.common.Status:
        # Logic to move the robot
        if self.finished:
            # If the robot has finished moving, return success
            self.node.get_logger().info("Robot has finished moving")
            return py_trees.common.Status.SUCCESS
        elif self.failure:
            # If the robot has failed to move, return failure
            self.node.get_logger().info("Robot failed to move")
            return py_trees.common.Status.FAILURE
        elif self.running:
            # If the robot is still moving, return running
            self.node.get_logger().info("Robot is still moving")
            return py_trees.common.Status.RUNNING
        else:
            # If the robot is not moving, return failure
            self.node.get_logger().info("Robot is not moving")
            # return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        pass


class BehaviorTreeNode(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('behavior_tree_node')
        self.get_logger().info("Initializing Behavior Tree")

        # Create and register the global blackboard
        self.blackboard = py_trees.blackboard.Client(name="Global")
        for key in ['/griper/state', '/griper/target', '/griper/position', '/griper/moving', '/mir/state', '/mir/position/x', '/mir/position/y', '/mir/position/w', '/mir/velocity/linear', '/mir/velocity/angular', '/mir/battery_percentage']:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
            self.blackboard.set(key, 0)

        # Create a subscriber for MiR messages
        self.mir_subscriber = self.create_subscription(MiRState, 'MiRState', self.mir_message_callback, 10)

        # Create a subscriber for gripper messages
        self.gripper_subscriber = self.create_subscription(InputMsg, 'Robotiq2FGripperRobotInput', self.gripper_message_callback, 10)
 
        # Create the gripper open and close behaviors
        # open_gripper = OpenGripper("Open griper", self)
        # close_gripper = CloseGripper("Close griper", self)
        # create the MiR send mission behavior
        mir_mission_to_robot = MiRMission(self, "MiRMoveToRobot", "7eaa508b-2f02-11f0-befc-000129af97ab")
        mir_mission_from_robot = MiRMission(self, "MiRMoveFromRobot", "5c5e1739-2f02-11f0-befc-000129af97ab")
        # create the robot move behaviors
        robot_to_mir_pos = RobotMove('Robot move over MiR', self, trajectories.TRAJECTORIES['to_mir_pos'])
        robot_close_grip_pos = RobotMove('Robot grip position', self, trajectories.TRAJECTORIES['to_grip_close_pos'])
        robot_from_mir_pos = RobotMove('Robot move from MiR', self, trajectories.TRAJECTORIES['from_mir_pos'])
        robot_to_pipe_pos = RobotMove('Robot move to pipe', self, trajectories.TRAJECTORIES['to_pipe_pos'])
        robot_open_grip_pos = RobotMove('Robot open gripper position', self, trajectories.TRAJECTORIES['to_grip_open_pos'])
        robot_home_pos = RobotMove('Robot home position', self, trajectories.TRAJECTORIES['from_pipe_pos'])

        robot_test1_move = RobotMove('Robot test move', self, trajectories.TRAJECTORIES['test1'])
        robot_test2_move = RobotMove('Robot test move', self, trajectories.TRAJECTORIES['test2'])

        # create a sequence node
        # self.sequence_place_pipe = py_trees.composites.Sequence(name="Sequence place pipe", memory=True, children=[robot_to_pipe_pos, robot_open_grip_pos, open_gripper, robot_home_pos])
        # self.sequence_pick_pipe = py_trees.composites.Sequence(name="Sequence pick pipe", memory=True, children=[robot_close_grip_pos, close_gripper, robot_from_mir_pos])
        # # create a parallel node
        # self.parallel_mir_to_pos = py_trees.composites.Parallel(name="Parallel get mir", policy=py_trees.common.ParallelPolicy.SuccessOnAll(), children=[mir_mission_to_robot, robot_to_mir_pos])
        # self.parallel_mir_from_pos = py_trees.composites.Parallel(name="Parallel remove mir", policy=py_trees.common.ParallelPolicy.SuccessOnAll(), children=[mir_mission_from_robot, self.sequence_place_pipe])
        # # create a selector node
        # self.selector = py_trees.composites.Selector(name="Selector", memory=True)
        # # create a decorator node
        # self.decorator = py_trees.decorators.FailureIsRunning(name="Decorator")
        # Create a root node
        # self.root = py_trees.composites.Sequence(name="Root", memory=True, children=[mir_mission_to_robot, close_gripper, mir_mission_from_robot, open_gripper])
        # self.root = py_trees.composites.Sequence(name="Root", memory=True, children=[mir_mission_from_robot, close_gripper, mir_mission_to_robot, open_gripper])
        # self.root = py_trees.composites.Sequence(name="Root", memory=True, children=[self.parallel_mir_to_pos, self.sequence_pick_pipe, self.parallel_mir_from_pos])
        self.root = py_trees.composites.Sequence(name="Root", memory=True, children=[mir_mission_to_robot, robot_to_mir_pos, robot_close_grip_pos, robot_from_mir_pos, robot_to_pipe_pos, robot_open_grip_pos, robot_home_pos, mir_mission_from_robot])


        # Build the behavior tree
        # Set up the behavior tree
        self.behaviour_tree = py_trees.trees.BehaviourTree(self.root) 
        # Set the root node as the behavior tree
        self.behaviour_tree.root = self.root
        print(py_trees.display.unicode_tree(self.behaviour_tree.root, show_status=True))
        print(py_trees.display.unicode_blackboard())
        print(self.blackboard)
        py_trees.display.render_dot_tree(self.root)

        # Create a timer to tick the behavior tree
        self.timer = self.create_timer(2, self.tick)
    
    def tick(self):
        # This method is called to tick the behavior tree
        self.get_logger().info("Ticking the Behavior Tree")
        self.get_logger().info(py_trees.display.unicode_tree(self.root, show_status=True))
        # print(py_trees.display.unicode_tree(self.root, show_status=True))
        self.behaviour_tree.tick()

    def mir_message_callback(self, msg):
        # Callback for MiR messages
        # self.get_logger().info(f"Received MiR message: {msg}")
        # Update the blackboard with the new message
        self.blackboard.mir.state = msg.state_id                            # MiR state (0 = none, 1 = starting, 2 = shutting down, 3 = ready, 4 = pause, 5 = executing, 6 = aborted, 7 = completed, 8 = docked, 9 = docking, 10 = emergency stop, 11 = manual control, 12 = error)
        self.blackboard.mir.position.x = msg.position.x                     # MiR position x
        self.blackboard.mir.position.y = msg.position.y                     # MiR position y    
        self.blackboard.mir.position.w = msg.position.theta                 # MiR position w
        self.blackboard.mir.velocity.linear = msg.velocity.linear           # MiR velocity linear
        self.blackboard.mir.velocity.angular = msg.velocity.angular         # MiR velocity angular
        self.blackboard.mir.battery_percentage = msg.battery_percentage     # MiR battery percentage

    def gripper_message_callback(self, msg):
        # Callback for gripper messages
        # self.get_logger().info(f"Received gripper message: {msg}")
        # Update the blackboard with the new message
        self.blackboard.griper.state = msg.g_sta        # gripper state (0 = is reset, 1 = activating, 3 = active)
        self.blackboard.griper.target = msg.g_pr        # gripper target (0 = open, 255 = closed)
        self.blackboard.griper.position = msg.g_po      # gripper position (0 = open, 255 = closed)
        self.blackboard.griper.moving = msg.g_obj       # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()