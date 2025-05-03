import rclpy
from mir_api import MiR_API as mir_api
import json
from rclpy.node import Node
from messages.msg import MiRState
from messages.srv import MirAppendMission

class MirNode(Node):
    def __init__(self):
        super().__init__('mir_node')
        self.declare_parameter('ip', '172.31.1.148')
        ip = self.get_parameter('ip').get_parameter_value().string_value
        self.mir = mir_api(ip)
        self.state_publisher = self.create_publisher(MiRState, 'MiRState', 10)
        self.append_mission_service = self.create_service(MirAppendMission, 'MiRAppendMission', self.append_mission_service_callback)

        self.get_logger().info('MirNode has been started')
        self.get_logger().info('Connecting to MiR at {}'.format(ip))

        self.publish_frequency = 10 # Hz
        self.publish_period = 1.0 / self.publish_frequency
        self.create_timer(self.publish_period, self.timer_callback)


    def timer_callback(self):
        msg = MiRState()
        state = self.mir.get_state()
        if state is None:
            self.get_logger().error('Failed to retrieve state from MiR API')

        try:
            # Handle string fields that might be None
            msg.mission_queue_url = state.get('mission_queue_url', '') or ''
            msg.mission_queue_id = state.get('mission_queue_id', 0) or 0
            msg.allowed_methods = state.get('allowed_methods', '') or ''

            # Handle user_prompt (convert dictionary to UserPrompt)
            user_prompt = state.get('user_prompt', {}) or {'has_request': False, 'guid': '', 'user_group': '', 'question': '', 'options': [], 'timeout': {'sec': 0, 'nanosec': 0}}
            msg.user_prompt.has_request = user_prompt.get('has_request', False)
            msg.user_prompt.guid = user_prompt.get('guid', '')
            msg.user_prompt.user_group = user_prompt.get('user_group', '')
            msg.user_prompt.question = user_prompt.get('question', '')
            msg.user_prompt.options = user_prompt.get('options', [])
            timeout = user_prompt.get('timeout', {})
            msg.user_prompt.timeout.sec = timeout.get('sec', 0)
            msg.user_prompt.timeout.nanosec = timeout.get('nanosec', 0)

            # Handle velocity (convert dictionary to Twist2D)
            velocity = state.get('velocity', {})
            msg.velocity.linear = velocity.get('linear', 0.0)
            msg.velocity.angular = velocity.get('angular', 0.0)

            # Handle position (convert dictionary to geometry_msgs/Pose2D)
            position = state.get('position', {})
            msg.position.x = position.get('x', 0.0)
            msg.position.y = position.get('y', 0.0)
            msg.position.theta = position.get('orientation', 0.0)

            # Handle other fields
            msg.joystick_low_speed_mode_enabled = state.get('joystick_low_speed_mode_enabled', False)
            msg.mode_id = state.get('mode_id', 0)
            msg.moved = state.get('moved', 0.0)
            msg.robot_name = state.get('robot_name', '')
            msg.joystick_web_session_id = state.get('joystick_web_session_id', '')
            msg.uptime = state.get('uptime', 0)
            msg.unloaded_map_changes = state.get('unloaded_map_changes', False)
            msg.distance_to_next_target = state.get('distance_to_next_target', 0.0)
            msg.serial_number = state.get('serial_number', '')
            msg.mode_key_state = state.get('mode_key_state', '')
            msg.battery_percentage = state.get('battery_percentage', 0.0)
            msg.map_id = state.get('map_id', '')
            msg.safety_system_muted = state.get('safety_system_muted', False)
            msg.mission_text = state.get('mission_text', '')
            msg.state_text = state.get('state_text', '')
            msg.footprint = state.get('footprint', '')
            msg.robot_model = state.get('robot_model', '')
            msg.mode_text = state.get('mode_text', '')
            msg.session_id = state.get('session_id', '')
            msg.state_id = state.get('state_id', 0)
            msg.battery_time_remaining = state.get('battery_time_remaining', 0)

        except Exception as e:
            self.get_logger().error(f"Error processing state: {e}")
        try:
            # self.get_logger().info(msg)
            self.state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing message: {e}")

    def append_mission_service_callback(self, request, response):
        response_json = self.mir.post_append_mission(request.mission_id)
        if response_json is None:
            self.get_logger().error('Failed to append mission')
            response.success = False
            response.response = 'Failed to append mission'
        else:
            self.get_logger().info(f"Mission {request.mission_id} appended successfully")
            response.success = True
            response.priority = response_json.get('priority', 0)
            response.response = 'Mission appended successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MirNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()