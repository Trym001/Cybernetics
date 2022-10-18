import datetime
import time
from socket import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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

empty_array_1 = []
empty_array_2 = []
acc_array = []
acc_array2 = []
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
    KF.update(meas_value=measurements[3], meas_value2=measurements[2], mean_variance=0.015)  # her brukes avstandsmåling for å estimere tilstandene
    #KF.update(meas_value=measurements[2], mean_variance=0.015)  # her brukes akselerasjon i z-retning for å estimere tilstandene

    estimates = KF.pos
    #log_measurements_and_estimates(delta_t, estimates, measurements)

    return estimates


def log_measurements_and_estimates(delta_t, estimates, measurements):
    sensor_log_data.append([delta_t, measurements[0], measurements[1], measurements[2], measurements[3]])
    estimates_log_data.append([estimates.item(0, 0), estimates.item(1, 0), estimates.item(2, 0)])

    if len(sensor_log_data) > 100:
        np.savetxt('measures.csv', sensor_log_data, delimiter=',')
        np.savetxt('estimates.csv', estimates_log_data, delimiter=',')
        sensor_log_data.clear()
        estimates_log_data.clear()


def arduino_has_been_reset():
    global reset
    if reset:
        print("Arduino is offline.. Resetting kalman filter")
        #global f
        #f = Fusion()
        reset = False


while counter < 500:
    #global counter
    sensor_values = arduino_send_receive(KF.pos)
    #print("raw sensor: ", sensor_values[3], " Estimates: ", KF.pos)
    #print(sensor_values[3])
    empty_array_1.append(KF.pos)
    empty_array_2.append(sensor_values[3])
    acc_array.append(KF.acc)
    acc_array2.append(sensor_values[2])
    # Set up plot to call animate() function periodically
    if sensor_values is not None:
        estimate(sensor_values)
    else:
        arduino_has_been_reset()

    counter += 1






plt.close(0); plt.figure(0)
#plt.subplot(2,1,1)
plt.plot(empty_array_1, 'r')
plt.plot(empty_array_2, 'b')
plt.grid(True)
#plt.subplot(2,1,2)
#plt.plot(acc_array, 'r')
#plt.plot(acc_array2, 'b')
#plt.grid(True)
#print("Predited",empty_array_1)
#t("Distance",empty_array_2)
#print(" Accel", acc_array2)

plt.show()


