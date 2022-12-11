import datetime
import time
from socket import *
import numpy as np
import matplotlib.pyplot as plt



from pythonFiles.internealLibrary.kalman_filter import KF

udp_socket = socket(AF_INET, SOCK_DGRAM)
udp_socket.settimeout(1)

np.set_printoptions(suppress=True)

arduino_ip = '192.168.10.240'
arduino_port = 8888

# En klasse som håndterer sensor fusion og bygger matrisene som brukes i Kalman filteret. KF er implementert som en egen klasse. Disse to må dere implementere selv
#f = Fusion()
KF = KF(initial_x=0.0, initial_v=0.0, initial_a=1, accel_variance= 1.9097)

reset = True
previous_time = datetime.datetime.now()

sensor_log_data = []
estimates_log_data = []

Pos_predict = []
Pos_sensor = []
Acc_predict = []
Acc_sensor = []
counter = 0


def arduino_send_receive(estimate):
    global reset
    global previous_time
    if not reset:
        previous_time = datetime.datetime.now()

    distance = KF.mean
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


def estimate(measurements):
    global previous_time
    now = datetime.datetime.now()
    diff = now - previous_time
    previous_time = now

    delta_t = diff.total_seconds()
    KF.predict(dt=delta_t)
    KF.update(meas_value=measurements[3], meas_value2=measurements[2], mean_variance2=0.015, mean_variance=1.9097)  # her brukes avstandsmåling for å estimere tilstandene

    estimates = KF.pos
    return estimates

def arduino_has_been_reset():
    global reset
    if reset:
        print("Arduino is offline.. Resetting kalman filter")
        reset = False

#collecting data from sensor/kalman filter
while counter < 500:
    sensor_values = arduino_send_receive(KF.pos)
    Pos_predict.append(KF.pos)
    Pos_sensor.append(sensor_values[3])
    Acc_predict.append(KF.acc)
    Acc_sensor.append(sensor_values[2])
    if sensor_values is not None:
        estimate(sensor_values)
    else:
        arduino_has_been_reset()

    counter += 1

#print(np.std(Pos_sensor))
#print(np.std(Acc_sensor))



#plotting data
plt.close(0); plt.figure(0)
plt.subplot(2,1,1)
plt.plot(Pos_predict, 'r', label ='Predicted')
plt.plot(Pos_sensor, 'b', label ='Measured')
plt.grid(True)
plt.xlabel("Position")
plt.legend(loc='lower right')
plt.subplot(2,1,2)
plt.plot(Acc_predict, 'r', label ='Predicted')
plt.plot(Acc_sensor, 'b', label ='Measured')
plt.xlabel('Acceleration')
plt.grid(True)
plt.legend(loc='lower right')

plt.tight_layout()
plt.show()


