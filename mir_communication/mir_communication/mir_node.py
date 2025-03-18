import rclpy
import mir_api
import json
from rclpy.node import Node
from messages.msg import MiRState

class MirNode(Node):
    def __init__(self, node_name, ip='172.31.1.148'):
        super().__init__('mir_node', ip='172.31.1.148')
        self.mir = mir_api.MirApi(ip)
        self.publisher = self.create_publisher(MiRState, 'MiRState', 10)

        self.get_logger().info('MirNode has been started')

        self.publish_frequency = 1 # Hz
        self.publish_period = 1.0 / self.publish_frequency
        self.create_timer(self.publish_period, self.timer_callback)


    def timer_callback(self):
        msg = MiRState()
        state = self.mir.get_state()
        for item in state:
            setattr(msg, item, state[item])
        
        self.publisher.publish(msg)


def JsonTopic(json_data, topic):
    msg = json.dumps(topic)




def main():
    rclpy.init()
    node = MirNode('mir_node')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()