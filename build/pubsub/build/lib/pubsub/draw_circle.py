import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class drawCircleNode(Node):

    def __init__(self):
        super().__init__('draw_circle')
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(self.send_vel_cmd)
        self.get_logger().info('Draw Circle node has been finished')

    def send_cel_cmd(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = drawCircleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
