import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import serial
import struct
import RPi.GPIO as GPIO


class My_vel_sub(Node):
    def __init__(self):
        super().__init__('My_vel_sub')
        self.subscription = self.create_subscription(
            Twist, '/turtle1/cmd_vel', self.listener_callback, 10)

        # send data to arduino through serial
        # self.serial_port = '/dev/ttyACM0'
        # self.baud_rate = 9600
        # self.ser = serial.Serial(self.serial_port, self.baud_rate)

        self.direction = [12, 13]
        self.speed = [14, 15]
        GPIO.setmode(GPIO.BOARD)
        # setup GPIO PINS
        GPIO.setup(self.direction[0], GPIO.OUT)
        GPIO.setup(self.direction[1], GPIO.OUT)
        GPIO.setup(self.speed[0], GPIO.OUT)
        GPIO.setup(self.speed[1], GPIO.OUT)

    def listener_callback(self, msg):
        linear_data = (msg.linear.x, msg.linear.y, msg.linear.z)
        angular_data = (msg.angular.x, msg.angular.y, msg.angular.z)

        self.get_logger().info(
            f'Linear{linear_data} Angular{angular_data}')
        if angular_data[2] == 0:
            if linear_data[0] > 0:
                GPIO.output(self.direction[0], GPIO.HIGH)
                GPIO.output(self.direction[1], GPIO.HIGH)
                GPIO.output(self.speed[0], GPIO.HIGH)
                GPIO.output(self.speed[1], GPIO.HIGH)
            elif linear_data[0] < 0:
                GPIO.output(self.direction[0], GPIO.LOW)
                GPIO.output(self.direction[1], GPIO.LOW)
                GPIO.output(self.speed[0], GPIO.HIGH)
                GPIO.output(self.speed[1], GPIO.HIGH)
        elif angular_data < 0:
            # move left
            GPIO.output(self.direction[0], GPIO.HIGH)
            GPIO.output(self.direction[1], GPIO.LOW)
            GPIO.output(self.speed[0], GPIO.HIGH)
            GPIO.output(self.speed[1], GPIO.HIGH)
        elif angular_data < 0:
            # move left
            GPIO.output(self.direction[0], GPIO.LOW)
            GPIO.output(self.direction[1], GPIO.HIGH)
            GPIO.output(self.speed[0], GPIO.HIGH)
            GPIO.output(self.speed[1], GPIO.HIGH)

        # send data to arduino through serial
        # data_to_send = struct.pack('ffffff',
        #                            msg.linear.x,
        #                            msg.linear.y,
        #                            msg.linear.z,
        #                            msg.angular.x,
        #                            msg.angular.y,
        #                            msg.angular.z)

        # # data_to_send = b''.join(data_to_send)
        # self.ser.write(data_to_send)

    def destroy_node(self):
        self.ser.close()
        self.get_logger().info('Serial Closed!!')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    my_velocity_subscriber = My_vel_sub()
    rclpy.spin(my_velocity_subscriber)

    my_velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
