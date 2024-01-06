import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class fix_vel_sub(Node):
    def __init__(self):
        super().__init__('My_vel_sub')
        self.subscription = self.create_subscription(
            Twist, 'twist', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Linear{msg.linear.x, msg.linear.y, msg.linear.z}, Angular{msg.angular.x, msg.angular.y, msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    fix_velocity_subscriber = fix_vel_sub()
    rclpy.spin(fix_velocity_subscriber)

    fix_velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
