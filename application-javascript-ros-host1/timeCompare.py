import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy.clock


class TimeChecker(Node):
    def __init__(self):
        super().__init__('time_checker')

        # Create a subscription to the odometry topic
        self.odom_sub = self.create_subscription(
            Odometry, '/ori_odom', self.odom_cb, 1
        )

    def odom_cb(self, msg):
        # Get the current ROS time
        current_time = self.get_clock().now().to_msg()

        # Compute the time difference
        print(current_time)
        print(msg.header.stamp)
        time_diff = (current_time.sec*1000000000 + current_time.nanosec) - (msg.header.stamp.sec*1000000000 + msg.header.stamp.nanosec)
        time_diff_sec = time_diff / 1000000000
        # Print the time difference
        self.get_logger().info(
            f"Time difference: {time_diff_sec} seconds"
        )


def main(args=None):
    rclpy.init(args=args)

    time_checker = TimeChecker()

    rclpy.spin(time_checker)

    time_checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()