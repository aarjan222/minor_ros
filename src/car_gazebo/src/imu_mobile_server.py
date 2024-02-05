#!/usr/bin/env python3
import socket
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3


imu_data = {
    'a': (0.0, 0.0, 0.0),
    'g': (0.0, 0.0, 0.0),
    'm': (0.0, 0.0, 0.0),
}


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.imupublisher_ = self.create_publisher(Imu, "car_imu_data", 10)
        self.magpublisher_ = self.create_publisher(
            MagneticField, "car_imu_mag", 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):

        msg = Imu()
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.orientation = Quaternion()
        msg.orientation.x = -1.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity = Vector3()
        msg.angular_velocity.x = float(imu_data['g'][0])
        msg.angular_velocity.y = float(imu_data['g'][1])
        msg.angular_velocity.z = float(imu_data['g'][2])

        msg.angular_velocity_covariance[0] = 0.1
        msg.angular_velocity_covariance[4] = 0.1
        msg.angular_velocity_covariance[8] = 0.1

        msg.linear_acceleration = Vector3()
        msg.linear_acceleration.x = float(imu_data['a'][0])
        msg.linear_acceleration.y = float(imu_data['a'][1])
        msg.linear_acceleration.z = float(imu_data['a'][2])

        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.imupublisher_.publish(msg)

        msg = MagneticField()
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.magnetic_field.x = float(imu_data['m'][0])
        msg.magnetic_field.y = float(imu_data['m'][1])
        msg.magnetic_field.z = float(imu_data['m'][2])

        self.magpublisher_.publish(msg)


def threaded_imu_loop():

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    server_address = '10.100.40.186'
    server_port = 8090

    server = (server_address, server_port)
    sock.bind(server)
    print("Listening on " + server_address + ":" + str(server_port))

    while True:
        payload, client_address = sock.recvfrom(1024)
        # print("payload aayox    ")
        vals = str(payload).split(',')

        if len(vals) == 5:
            t, _, ax, ay, az = vals
            az = az.replace("'", "")
            imu_data['a'] = (ax, ay, az)
        elif len(vals) == 9:
            t, _, ax, ay, az, _, gx, gy, gz = vals
            gz = gz.replace("'", "")
            imu_data['a'] = (ax, ay, az)
            imu_data['g'] = (gx, gy, gz)
        elif len(vals) == 13:
            t, _, ax, ay, az, _, gx, gy, gz, _, mx, my, mz = vals
            mz = mz.replace("'", "")
            imu_data['a'] = (ax, ay, az)
            imu_data['g'] = (gx, gy, gz)
            imu_data['m'] = (mx, my, mz)
        else:
            pass
        # print(imu_data)


def main(args=None):
    imu_thread = threading.Thread(target=threaded_imu_loop)
    imu_thread.start()
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
