#!/usr/bin/env python3
from crc8 import *
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import serial
import struct
import math
import numpy as np
from geometry_msgs.msg import Pose
import socket
import threading
import time


# this contains the socket: client model which sends data from stm to ros to tcp
# server model is the ros2_control bicycle plugin,
# see there and set same port and server ip in plugin server model also

PORT = 5555
SERVER_IP = "127.0.0.1"

START_BYTE = bytes.fromhex('A5')
DATA_LENGTH = 27

# rear_enc=0
# rear_vel=0
# front_steering=0


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
        global rear_enc, rear_vel, front_steering
        msg = Pose()
        while True:
            ret, data = self.read()

            if data is None:
                break
            if not ret:
                break
            if (len(data) != DATA_LENGTH):
                break

            data = struct.unpack('=cffffffcc', data)
            start_byte, x_, y_, theta_, rear_enc, rear_vel, front_steering, hash_byte, newline_byte = data

            # if pi not connected to stm: send random feedback data for now
            # x_ = 1.3
            # y_ = 2.8
            # theta_ = 0.5

            # rear_enc = 300.2
            # rear_vel = 4
            # front_steering = 0.05
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


def threaded_feedback_loop():
    global rear_enc, rear_vel, front_steering
    # 1. connect to the socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, PORT))
    print("Socket created.")

    feed_back_data = [1232.23, 5, 0.05]
    print("Connected to server.")

    while (1):
        # 2. first send all data from client to server
        # always send all data, from start to end for frame matching
        send_buffer = bytearray()

        for val in feed_back_data:
            send_buffer.extend(struct.pack('f', val))

        # client sends data, server must listen the data at same time, which is done
        # inside read() in plugin
        client_socket.sendall(send_buffer)

        # 3. also receive data from server
        receive_buffer = client_socket.recv(8)
        # for now only 2 data lin_x and ang_z datas from ros2_control
        commanded_data_by_ros2 = struct.unpack('ff', receive_buffer)

        # in this order
        # rear_wheel_traction  / rear_wheel_velocity
        # front_wheel_steering / front_wheel_position
        # print commanded data by ros2
        print("vel=", commanded_data_by_ros2[0],
              "\tpos=", commanded_data_by_ros2[1])

        # time.sleep(0.1)


def main(args=None):
    feedback_thread = threading.Thread(target=threaded_feedback_loop)
    feedback_thread.start()

    rclpy.init(args=args)
    my_velocity_subscriber = My_vel_sub()
    rclpy.spin(my_velocity_subscriber)

    my_velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
