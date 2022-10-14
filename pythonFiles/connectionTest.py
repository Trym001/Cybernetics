import datetime
import time
from socket import *
import numpy as np


udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress=True)

arduino_ip = '192.168.10.240'
arduino_port = 8888


reset = True
previous_time = datetime.datetime.now()

sensor_log_data = []
estimates_log_data = []


def arduino_send_receive():
    global reset
    global previous_time
    if not reset:
        previous_time = datetime.datetime.now()

    distance = 0
    udp_socket.sendto(str(distance).encode(), (arduino_ip, arduino_port))
    try:
        inbound_message, remote_address = udp_socket.recvfrom(24)
        # returns an array with the following values
        # [accel_x, accel_y, accel_z, range_sensor]
        parse = np.array(inbound_message.decode('ascii').split(',')).astype(float)
        reset = True
        ret = [parse.item(0), parse.item(1), parse.item(2),
               parse.item(3)]  # husk at verdiene har ulike enheter (m/s^2 og mm)
        return ret
    except Exception as e:
        print(e)


def arduino_has_been_reset():
    global reset
    if reset:
        print("Arduino is offline.. Resetting imaginary kalman filter")
        reset = False


while True:
    sensor_values = arduino_send_receive()
    if sensor_values is not None:
        print(sensor_values) # standard avvik 1.1cm paa avstandssensor.
        pass
    else:
        arduino_has_been_reset()
