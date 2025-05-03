import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.action import ActionClient
import py_trees
import py_trees_ros
import behavior_tree
import time
from messages.msg import MiRState
from messages.srv import MirAppendMission
from messages.msg import Robotiq2FGripperRobotInput as InputMsg
from messages.msg import Robotiq2FGripperRobotOutput as OutputMsg
from messages.action import Moveit
    
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
        msg.r_fr = 255
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
    def __init__(self, name, node, position):
        super(RobotMove, self).__init__(name=name)
        self.node = node
        self.robot_action = None
        self.position = position
        self.running = False
        self.finished = False

    def initialise(self):
        # Initialize the blackboard
        self.blackboard = py_trees.blackboard.Client(name='MiRBlackboard')
        for key in ['/mir/state', '/mir/position/x', '/mir/position/y', '/mir/position/w', '/mir/velocity/linear', '/mir/velocity/angular', '/mir/battery_percentage', '/griper/state', '/griper/target', '/griper/position', '/griper/moving']:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        print(self.blackboard)
        # Initialize the robot client
        self.node.get_logger().info("Initializing robot action client")
        self.robot_action = ActionClient(self.node, Moveit, 'robot_moveit')
        # Wait for the action to be available
        self.robot_action.wait_for_server()
        self.node.get_logger().info("Robot action client initialized")
        # Create a message
        pos_msg = Moveit.Goal()
        pos_msg.pose = self.position
        # Send the goal to the robot
        self.node.get_logger().info("Sending goal to robot")
        pos_msg = Moveit.Goal()
        pos_msg.pose = self.position
        send_future = self.robot_action.send_goal_async(pos_msg)
        send_future.add_done_callback(self.goal_response_callback)
        self.node.get_logger().info("Goal sendt to robot")

    def goal_response_callback(self, future):
        # Callback for the goal response
        goal_handle = future.result()
        self.running = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        # Callback for the result
        result = future.result()
        self.finished = True
        self.running = False

    def update(self) -> py_trees.common.Status:
        # Logic to move the robot
        if self.finished:
            # If the robot has finished moving, return success
            self.node.get_logger().info("Robot has finished moving")
            return py_trees.common.Status.SUCCESS
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
        open_gripper = OpenGripper("Open griper", self)
        close_gripper = CloseGripper("Close griper", self)
        # create the MiR send mission behavior
        mir_mission_to_robot = MiRMission(self, "MiRMoveToRobot", "bc0d09ca-274c-11f0-82ff-000129af97ab")
        mir_mission_from_robot = MiRMission(self, "MiRMoveFromRobot", "a563317e-da62-11ef-b29c-000129af97ab")

        # create a parallel node
        # self.parallel = py_trees.composites.Parallel(name="Parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll(), children=[open_gripper, mir_mission_from_robot])
        # # create a selector node
        # self.selector = py_trees.composites.Selector(name="Selector", memory=True)
        # # create a decorator node
        # self.decorator = py_trees.decorators.FailureIsRunning(name="Decorator")
        # Create a root node
        # self.root = py_trees.composites.Sequence(name="Root", memory=True, children=[mir_mission_to_robot, close_gripper, mir_mission_from_robot, open_gripper])
        self.root = py_trees.composites.Sequence(name="Root", memory=True, children=[mir_mission_from_robot, close_gripper, mir_mission_to_robot, open_gripper])


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