from messages.srv import MirAppendMission
import sys
import rclpy
from rclpy.node import Node

class SendService(Node):
    def __init__(self):
        super().__init__('send_service')
        self.cli = self.create_client(MirAppendMission, 'MiRAppendMission')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = MirAppendMission.Request()

    def send_request(self):
        mission_id = 'bc0d09ca-274c-11f0-82ff-000129af97ab'
        self.req.mission_id = mission_id
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    send_service = SendService()
    send_service.send_request()

    while rclpy.ok():
        rclpy.spin_once(send_service)
        if send_service.future.done():
            try:
                response = send_service.future.result()
            except Exception as e:
                send_service.get_logger().info('Service call failed %r' % (e,))
            else:
                send_service.get_logger().info('Result: %s' % response.response)
            break

    send_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()