import serial
from time import sleep
ser = serial.Serial('/dev/ttyS0',
        9600,
        timeout = 1
        )


def read_serial_data():
    x = ser.readline()
    return x
