import time
import rclpy
from rclpy.node import Node
from messages.msg import Robotiq2FGripperRobotOutput as OutputMsg

class TestNode(Node):
    def __init__(self):
        super().__init__('open_close_node')
        self.publisher = self.create_publisher(OutputMsg, 'Robotiq2FGripperRobotOutput', 10)

    def send_gripper_open_command(self):
        msg = OutputMsg()
        msg.rPR = 0
        msg.rSP = 127
        msg.rFR = 127
        self.publisher.publish(msg)

    def send_gripper_close_command(self):
        msg = OutputMsg()
        msg.rPR = 255
        msg.rSP = 127
        msg.rFR = 127
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = TestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    #while not rclpy.shutdown():
    node.send_gripper_close_command()
    #time.sleep(2)
    #node.send_gripper_open_command()
    #time.sleep(2)


    node.gripper.disconnect()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()