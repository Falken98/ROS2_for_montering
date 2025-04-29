import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import behavior_tree
from messages.msg import MiRState
from messages.srv import MirAppendMission
from messages.msg import Robotiq2FGripperRobotInput as InputMsg

class Blackboard:
    def __init__(self):
        # Robotiq gripper
        self.griper.state = None
        self.griper.target = None
        self.griper.position = None
        self.blackboard.gripper.moving = None
        # MiR robot
        self.mir.state = None
        self.mir.position.x = None
        self.mir.position.y = None
        self.mir.position.w = None
        self.mir.velocity.linear = None
        self.mir.velocity.angular = None
        self.mir.battery_percentage = None
    


class BehaviorTreeNode(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('example_behavior_tree')
        self.get_logger().info("Initializing Behavior Tree")

        # Create a blackboard
        self.blackboard = Blackboard()

        # Create a subscriber for MiR messages
        self.mir_subscriber = self.create_subscription(MiRState, 'MiRState', self.mir_message_callback)

        # Create a subscriber for gripper messages
        self.gripper_subscriber = self.create_subscription(InputMsg, 'Robotiq2FGripperRobotInput', self.gripper_message_callback)

        # Create a service client for MiR mission appending
        self.mir_client = self.create_client(MirAppendMission, 'mir_append_mission')
        # Wait for the service to be available
        while not self.mir_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Create a root node
        root = py_trees.composites.Sequence(name="Root", memory=True)

        # Print the tree structure
        py_trees.display.print_ascii_tree(root)

        # Set up the behavior tree
        behaviour_tree = py_trees.trees.BehaviourTree(root)

        # Create a timer to tick the behavior tree
        self.timer = self.create_timer(1.0, self.tick)
    
    def tick(self):
        # This method is called to tick the behavior tree
        self.get_logger().info("Ticking the Behavior Tree")
        self.behaviour_tree.tick()

    def mir_message_callback(self, msg):
        # Callback for MiR messages
        self.get_logger().info(f"Received MiR message: {msg}")
        # Update the blackboard with the new message
        self.blackboard.mir.state = msg.state
        self.blackboard.mir.position.x = msg.position.x
        self.blackboard.mir.position.y = msg.position.y
        self.blackboard.mir.position.w = msg.position.theta
        self.blackboard.mir.velocity.linear = msg.velocity.linear
        self.blackboard.mir.velocity.angular = msg.velocity.angular
        self.blackboard.mir.battery_percentage = msg.battery_percentage

    def gripper_message_callback(self, msg):
        # Callback for gripper messages
        self.get_logger().info(f"Received gripper message: {msg}")
        # Update the blackboard with the new message
        self.blackboard.gripper.state = msg.g_sta
        self.blackboard.gripper.target = msg.g_pr
        self.blackboard.gripper.position = msg.g_po
        self.blackboard.gripper.moving = msg.g_obj != 0

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()