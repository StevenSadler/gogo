#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from gogo_interfaces.msg import ModeSelect

class XboxTwist(Node):
    def __init__(self):
        super().__init__('xbox_twist')

        # QoS for high-rate topics (cmd_vel)
        cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # QoS for mode selection
        mode_select_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.pub = self.create_publisher(Twist, 'cmd_vel', cmd_vel_qos)
        self.mode_pub = self.create_publisher(ModeSelect, 'mode_select', mode_select_qos)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, cmd_vel_qos)


        # Parameters for mapping
        self.declare_parameter('enable_button', 4)  # LB
        self.declare_parameter('axis_linear_x', 1)  # left stick vertical
        self.declare_parameter('axis_angular_z', 3) # right stick horizontal
        self.declare_parameter('scale_linear', 0.10)
        self.declare_parameter('scale_angular', 0.72)

        self.enable_button = self.get_parameter('enable_button').value
        self.axis_linear = self.get_parameter('axis_linear_x').value
        self.axis_angular = self.get_parameter('axis_angular_z').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value

        # Mode buttons (configured via params)
        self.declare_parameter('mode_idle_button', 0)
        self.declare_parameter('mode_test_button', 1)
        self.declare_parameter('mode_drive_button', 2)

        self.mode_idle_button = self.get_parameter('mode_idle_button').value
        self.mode_test_button = self.get_parameter('mode_test_button').value
        self.mode_drive_button = self.get_parameter('mode_drive_button').value

        # For edge detection
        self.prev_enable = False
        self.prev_buttons = None
        self.last_mode = None

        # map mode names to button indexes
        self.mode_buttons = [
            ("MODE_IDLE", self.mode_idle_button),
            ("MODE_TEST", self.mode_test_button),
            ("MODE_DRIVE", self.mode_drive_button)
        ]

        self.get_logger().warn(f"PARAMS: scale_linear={self.scale_linear}, scale_angular={self.scale_angular}")
    
    def get_mode_selection(self, msg: Joy):
        """
        Return selected mode based on button presses:
        No buttons       -> None (do nothing)
        1 button         -> First pressed button mode
        Multiple buttons -> MODE_IDLE (for safety)
        """
        first_pressed = None
        for mode_name, button_index in self.mode_buttons:
            if msg.buttons[button_index]:
                if first_pressed is not None:
                    # multiple buttons pressed -> return MODE_IDLE, no log needed
                    return "MODE_IDLE"
                
                first_pressed = mode_name
        return first_pressed

    def joy_callback(self, msg: Joy):
        enable = msg.buttons[self.enable_button]

        # Rising or steady edge: actively controlling
        if enable:
            # Twist mapping
            twist = Twist()
            if msg.buttons[self.enable_button]:
                twist.linear.x = msg.axes[self.axis_linear] * self.scale_linear
                twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            self.pub.publish(twist)
        
        # Falling edge: deadman just released -> immediate stop
        elif self.prev_enable and not enable:
            self.pub.publish(Twist())  # single stop command
        
        # else: silence

        self.prev_enable = enable

        # Mode selection
        mode_selection = self.get_mode_selection(msg)
        if mode_selection is not None and mode_selection != self.last_mode:
            mode_msg = ModeSelect()
            mode_msg.mode = mode_selection
            self.mode_pub.publish(mode_msg)
            self.last_mode = mode_selection
            self.get_logger().warn(f"Mode selection: {mode_selection}")
        

def main(args=None):
    rclpy.init(args=args)
    node = XboxTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
