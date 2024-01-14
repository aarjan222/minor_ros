import serial
from time import sleep
import struct

from crc8 import *

START_BYTE = bytes.fromhex('A5') 
lin_x = 0.
ang_z = 0.

ser = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0', 9600)  # Open port with baud rate
while True:
    data_to_send = [bytes(struct.pack("f", lin_x)),
                    bytes(struct.pack("f", ang_z))]
    data_to_send = b''.join(data_to_send)

    data_to_send = bytes(data_to_send)
    hash_func = crc8()
    hash_func.update(data_to_send)
    data_to_send = START_BYTE + data_to_send + \
        bytes(struct.pack("c", hash_func.digest()))
  
    print(data_to_send)
    # for b in data_to_send:
    #     print(b)
    print(f"{len(data_to_send)=}")
    ser.write(data_to_send)
    sleep(1)
