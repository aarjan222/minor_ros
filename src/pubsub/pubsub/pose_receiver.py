from pubsub.crc8 import *
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import serial
import struct
import math
import numpy as np
from geometry_msgs.msg import Pose

START_BYTE = bytes.fromhex('A5')
DATA_LENGTH = 15


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class My_vel_sub(Node):
    def __init__(self):
        super().__init__('my_car_pose')

        # read data from stm through serial
        self.serial_port = "/dev/ttyS0"
        self.baud_rate = 9600
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        self.car_pose_ = self.create_publisher(Pose, 'car_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Pose()
        while True:
            ret, data = self.read()

            if data is None:
                break
            if not ret:
                break
            if (len(data) != DATA_LENGTH):
                break

            data = struct.unpack('=cfffcc', data)
            start_byte, x_, y_, theta_, hash_byte, newline_byte = data
            print(f"x y theta is {x_} {y_} {theta_}")

            q = quaternion_from_euler(0, 0, theta_)
            msg.position.x = x_
            msg.position.y = y_
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]

            self.car_pose_.publish(msg)
            self.get_logger().info(f"x y theta is {x_} {y_} {theta_}")
            self.i += 1

    def read(self):
        while (self.ser.inWaiting() > DATA_LENGTH):
            data_str = self.ser.readline()
            data_str = bytes(data_str)
            if len(data_str) < DATA_LENGTH:
                continue
            data_to_check_hash_for = data_str[1:-2]

            received_hash = data_str[-2]
            hash_func = crc8()
            hash_func.update(data_to_check_hash_for)

            if hash_func.digest()[0] == received_hash:
                return True, data_str
            print(f"Hash fail: {received_hash} {hash_func.digest()[0]}")
        return False, None

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
