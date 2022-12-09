import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from depthai_ros_msgs.msg import SpatialDetectionArray

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# Direction of target
class Direction(Enum):
    FORWARD = 0
    FORWARD_LEFT = 1
    FORWARD_RIGHT = 2
    LEFT = 3
    RIGHT = 4
    UNKNOWN = 5


class FollowBot(Node):
    image_width = 300
    image_height = 300
    fps = 15
    confidence = 0.80

    is_docked = False

    target = None
    target_direction = Direction.UNKNOWN
    target_distance = 0.0
    last_target_direction = Direction.UNKNOWN
    state = State.SEARCHING

    forward_margin_px = 20
    forward_turn_margin_px = 75
    reverse_thresh = 1.2
    follow_thresh = 1.6
    thresh_hysteresis = 0.05

    def __init__(self):
        super().__init__('followbot')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_system_default)

        self.original_odom = self.create_publisher(Odometry, 'ori_odom', qos)

    # Send cmd_vel to Create 3
    def drive(self, linear_x, angular_z):
        msg = Twist()
        msg.angular.z = angular_z
        msg.linear.x = linear_x

        self.cmd_vel_pub.publish(msg)

    # Calculate direction of target relative to robot
    def setTargetDirection(self):
        # No target found
        if self.target is None:
            self.target_direction = Direction.UNKNOWN
            return

        # Last known direction of target
        if self.target_direction is not Direction.UNKNOWN:
            self.last_target_direction = self.target_direction

        # Get distance of target to center of image
        center_dist = self.target.bbox.center.x - self.image_width / 2

        # Find target direction based on position in image
        if abs(center_dist) < self.forward_margin_px:
            self.target_direction = Direction.FORWARD
        elif abs(center_dist) < self.forward_turn_margin_px:
            if center_dist >= 0.0:
                self.target_direction = Direction.FORWARD_RIGHT
            else:
                self.target_direction = Direction.FORWARD_LEFT
        else:
            if center_dist >= 0.0:
                self.target_direction = Direction.RIGHT
            else:
                self.target_direction = Direction.LEFT
