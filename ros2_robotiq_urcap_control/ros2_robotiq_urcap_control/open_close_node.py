import time
import rclpy
from rclpy.node import Node
from messages.msg import Robotiq2FGripperRobotOutput as OutputMsg

class TestNode(Node):
    def __init__(self):
        super().__init__('open_close_node')
        self.publisher = self.create_publisher(OutputMsg, 'Robotiq2FGripperRobotOutput', 10)

    def send_gripper_open_command(self):
        self.get_logger().info('Sending open command')
        msg = OutputMsg()
        msg.r_pr = 0
        msg.r_sp = 127
        msg.r_fr = 127
        self.publisher.publish(msg)

    def send_gripper_close_command(self):
        self.get_logger().info('Sending close command')
        msg = OutputMsg()
        msg.r_pr = 255
        msg.r_sp = 127
        msg.r_fr = 127
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = TestNode()

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass

    while rclpy.ok(): # not rclpy.shutdown():
        node.send_gripper_close_command()
        time.sleep(2)
        node.send_gripper_open_command()
        time.sleep(2)

    node.gripper.disconnect()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()