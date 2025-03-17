import rclpy
from rclpy.node import Node
import mir_api


class MirNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.mir = mir_api.MirApi()

        self.get_logger().info('MirNode has been started')
        self.create_timer(1.0, self.timer_callback)


    def timer_callback(self):
        state = self.mir.get_state()
        


def main():
    rclpy.init()
    node = MirNode('mir_node')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()