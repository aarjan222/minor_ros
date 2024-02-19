#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Header


class TeleopToCarlikeBot(Node):
    def __init__(self):
        super().__init__('teleop_to_carlikebot')
        self.publisher_ = self.create_publisher(
            TwistStamped, '/bicycle_steering_controller/reference', 10)
        self.get_logger().info("Init")
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg: Twist):
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header = Header()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        # twist_stamped_msg.twist = msg*0.1
        twist_stamped_msg.twist.linear.x = 0.1 * msg.linear.x
        twist_stamped_msg.twist.angular.z = 0.1 * msg.angular.z
        # Republish the TwistStamped message to the /bicycle_steering_controller/reference topic
        self.publisher_.publish(twist_stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_to_carlikebot = TeleopToCarlikeBot()
    rclpy.spin(teleop_to_carlikebot)
    teleop_to_carlikebot.destroy_node()
    rclpy.shutdown()


main()
