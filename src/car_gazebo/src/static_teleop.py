#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class static_teleop(Node):
    def __init__(self):
        super().__init__('static_teleop')
        self.get_logger().info("Init")
        self.publisher_ = self.create_publisher(
            TwistStamped, '/bicycle_steering_controller/reference', 10)
        self.timer = self.create_timer(1.0/30.0, self.publish_twist)

    def publish_twist(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.5
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.1
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    st = static_teleop()
    rclpy.spin(st)
    st.destroy_node()
    rclpy.shutdown()


main()
