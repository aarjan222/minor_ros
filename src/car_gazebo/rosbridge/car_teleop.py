#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import serial
import struct
START_BYTE = bytes.fromhex('A5')
from crc8 import *

class My_vel_sub(Node):
    def __init__(self):
        super().__init__('car_control')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)
        
        self.get_logger().info('Init')
        
        # send data to stm through serial
        # gpio14 = txd to stm pa3
        # gpio15 = rxd to stm pa2
        self.serial_port = "/dev/ttyS0"
        self.baud_rate = 9600
        self.ser = serial.Serial(self.serial_port, self.baud_rate)


    def listener_callback(self, msg):
        lin_x = msg.linear.x
        ang_z = msg.angular.z
        # print(lin_x, ang_z)
        # self.get_logger().info('Linear X: {}, Angular Z: {}'.format(lin_x, ang_z))
        data_to_send = [struct.pack('ff', msg.linear.x, msg.angular.z)]
        message = b''.join(data_to_send)

        data_to_send = bytes(message)
        hash_func = crc8()
        hash_func.update(data_to_send)
        data_to_send = START_BYTE + data_to_send + \
            bytes(struct.pack("c", hash_func.digest()))

        self.get_logger().info(data_to_send)
        self.ser.write(data_to_send)
        print(data_to_send)

    def destroy_node(self):
        self.ser.close()
        self.get_logger().info('Serial Closed!!')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    car_control = My_vel_sub()
    rclpy.spin(car_control)

    car_control.destroy_node()
    rclpy.shutdown()



main()