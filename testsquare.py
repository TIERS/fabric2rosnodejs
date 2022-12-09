from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
import rclpy
import numpy as np

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('Minimal_Subscriber')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.original_odom = self.create_publisher(Odometry, 'ori_odom', qos)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def odom_callback(self, msg):
        self.original_odom.publish(msg)

        twist = Twist()
        for i in range(4):
        	twist.linear.x = i+1.1
        	twist.angular.z = -1.1+i
        	print(twist)
        	self.cmd_vel_pub.publish(twist)


def main(args=None):
	rclpy.init(args=args)
	
	minimal_subscriber = MinimalSubscriber()
	rclpy.spin(minimal_subscriber)

	minimal_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()