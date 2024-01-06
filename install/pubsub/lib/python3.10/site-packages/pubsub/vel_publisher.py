import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist


class fix_vel_pub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'twist', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()

        msg.linear.x = 1.0
        msg.linear.y = 2.0
        msg.linear.z = 3.0

        msg.angular.x = 1.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: %s' % msg)


def main(args=None):
    rclpy.init(args=args)
    fix_velocity_publisher = fix_vel_pub()
    rclpy.spin(fix_velocity_publisher)
    fix_velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
