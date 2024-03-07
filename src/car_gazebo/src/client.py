import time
import socket
import struct

PORT = 36258
SERVER_IP = "127.0.0.1"


def main():
    # 1. connect to the socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_IP, PORT))
    print("Socket created.")

    while True:
        # 2. first send all data from client to server
        feed_back_data = [10.54560, 20.06546, 30.89780]
        print("Connected to server.")

        # pack all sending data together in bytes and send data byte by byte
        for byte_data in feed_back_data:
            client_socket.sendall(struct.pack('f', byte_data))
        print("Data sent to server:", feed_back_data)

        # 3. also receive data from server
        commanded_data_by_ros2 = client_socket.recv(8)
        commanded_data_by_ros2 = struct.unpack('ff', commanded_data_by_ros2)

        # in this order
        # rear_wheel_traction  / rear_wheel_velocity
        # front_wheel_steering / front_wheel_position
        # print commanded data by ros2
        for i, value in enumerate(commanded_data_by_ros2):
            print(f"joint_commands[{i}] = {value:.2f}")
            
        time.sleep(1)


if __name__ == "__main__":
    main()
