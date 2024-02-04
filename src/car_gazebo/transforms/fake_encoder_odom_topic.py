#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class Encoder_Odom(Node):
    def __init__(self):
        super().__init__('encoder_odom')
        self.publisher_ = self.create_publisher(Pose, 'encoder_odom', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        msg.position.x = 1.0
        msg.position.y = 2.0
        msg.position.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: ')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    encoder_odom = Encoder_Odom()
    rclpy.spin(encoder_odom)
    encoder_odom.destroy_node()
    rclpy.shutdown()


main()
