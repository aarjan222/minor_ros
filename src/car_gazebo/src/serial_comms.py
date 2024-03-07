#!/usr/bin/env python
import os
import serial
import struct
import sys
import time

from crc8 import *

START_BYTE = bytes.fromhex('A5')
DATA_LENGTH = 27


class CarSerialComms:
    def __init__(self, serial_port, baudrate, logger=None):
        self.serial = serial.Serial(serial_port, baudrate)
        self.hash_func = crc8()
        self.logger = logger

    def _send(self, message):
        # print(f"Message: {''.join(format(x, '02x') for x in message)}")
        data_to_send = bytes(message)
        hash_func = crc8()
        hash_func.update(data_to_send)
        data_to_send = START_BYTE + data_to_send + \
            bytes(struct.pack("c", hash_func.digest()))
        # if self.logger is not None:
        #     self.logger.info(f"Sent {len(data_to_send)} bytes: {''.join(format(x, '02x') for x in data_to_send)}")
        self.serial.write(data_to_send)

    def read(self):
        while (self.serial.inWaiting() > DATA_LENGTH):
            data_str = self.serial.readline()
            data_str = bytes(data_str)
            if len(data_str) < DATA_LENGTH:
                continue
            # skip the start byte, hash byte and newline
            data_to_check_hash_for = data_str[1:-2]

            received_hash = data_str[-2]
            hash_func = crc8()
            hash_func.update(data_to_check_hash_for)

            if hash_func.digest()[0] == received_hash:
                return True, data_str
            print(f"Hash fail: {received_hash} {hash_func.digest()[0]}")

        return False, None

    def __del__(self):
        self.serial.close()
