import time
from serial import Serial
import serial
from matplotlib import pyplot as plt
import numpy as np
import math
import requests
import signal
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.IN)
GPIO.setup(12, GPIO.OUT)

GPIO.output(12, GPIO.LOW)

def initial_cond():
    # Initialize a list to store the time values
    global time_val
    time_val = []

    # Initialize variables to store the values

    # For Option 1
    # Initialize variables to store linear acceleration data for three axes
    global acc_x_val, acc_y_val, acc_z_val
    acc_x_val = []
    acc_y_val = []
    acc_z_val = []

    # Initialize variables to store angular velocity data for three axes
    global ang_x_val, ang_y_val, ang_z_val
    ang_x_val = []
    ang_y_val = []
    ang_z_val = []

    # Initialize variables to store magnetometer data for three axes
    global mag_x_val, mag_y_val, mag_z_val
    mag_x_val = []
    mag_y_val = []
    mag_z_val = []

    global acc_magnitude_val, ang_magnitude_val
    acc_magnitude_val = []
    ang_magnitude_val = []

    # For Option 2
    # Initialize variables to store temperature data
    global temp_accel_val, temp_gyro_x_val, temp_gyro_y_val, temp_gyro_z_val
    temp_accel_val = []
    temp_gyro_x_val = []
    temp_gyro_y_val = []
    temp_gyro_z_val = []

    # For Option 3
    # Initialize variables to store raw linear acceleration data for three axes
    global raw_acc_x_val, raw_acc_y_val, raw_acc_z_val
    raw_acc_x_val = []
    raw_acc_y_val = []
    raw_acc_z_val = []

    # Initialize variables to store raw angular velocity data for three axes
    global raw_ang_x_val, raw_ang_y_val, raw_ang_z_val
    raw_ang_x_val = []
    raw_ang_y_val = []
    raw_ang_z_val = []

    # For Option 4
    # Initialize variables to store gyro stabilized linear acceleration data for three axes
    global stab_acc_x_val, stab_acc_y_val, stab_acc_z_val
    stab_acc_x_val = []
    stab_acc_y_val = []
    stab_acc_z_val = []

    # Initialize variables to store gyro stabilized magnetometer data for three axes
    global stab_mag_x_val, stab_mag_y_val, stab_mag_z_val
    stab_mag_x_val = []
    stab_mag_y_val = []
    stab_mag_z_val = []

    # For Option 5
    # Initialize variables to store delta linear acceleration data for three axes
    global del_acc_x_val, del_acc_y_val, del_acc_z_val
    del_acc_x_val = []
    del_acc_y_val = []
    del_acc_z_val = []

    # Initialize variables to store delta angular velocity data for three axes
    global del_ang_x_val, del_ang_y_val, del_ang_z_val
    del_ang_x_val = []
    del_ang_y_val = []
    del_ang_z_val = []

    # For Option 6
    # Initialize variables to store Euler angles
    global euler_roll_val, euler_pitch_val, euler_yaw_val
    euler_roll_val = []
    euler_pitch_val = []
    euler_yaw_val = []

    # For Option 7
    # Initialize variables to store Orientation Matrix elements
    global M1_1_val, M1_2_val, M1_3_val
    M1_1_val = []
    M1_2_val = []
    M1_3_val = []

    global M2_1_val, M2_2_val, M2_3_val
    M2_1_val = []
    M2_2_val = []
    M2_3_val = []

    global M3_1_val, M3_2_val, M3_3_val
    M3_1_val = []
    M3_2_val = []
    M3_3_val = []

    # For Option 8
    # Initialize variables to store Attitude Update Matrix elements
    global C1_1_val, C1_2_val, C1_3_val
    C1_1_val = []
    C1_2_val = []
    C1_3_val = []

    global C2_1_val, C2_2_val, C2_3_val
    C2_1_val = []
    C2_2_val = []
    C2_3_val = []

    global C3_1_val, C3_2_val, C3_3_val
    C3_1_val = []
    C3_2_val = []
    C3_3_val = []


initial_cond()
global sample_time_ref
sample_time_ref = None
global start_flag
start_flag = False

choice = "opt1"

ser = Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)


def concatenate_hex_bytes(data):
    # Extract bytes 1-43 from data
    bytes = data[0:43]

    # Convert each byte to hexadecimal string
    hex_strings = [format(b, '02x') for b in bytes]

    # Concatenate hex strings
    hex_string = ''.join(hex_strings)
    return hex_string


def hex_to_float(hex_string):
    # Convert hex string to binary string
    bin_string = bin(int(hex_string, 16))[2:].zfill(32)

    # Extract sign, exponent, and mantissa from binary string
    sign = -1.0 if bin_string[0] == '1' else 1.0
    exponent = int(bin_string[1:9], 2) - 127
    mantissa = int(bin_string[9:], 2)

    # Calculate float representation
    float_num = sign * (1.0 + mantissa / 2 ** 23) * 2 ** exponent

    return float_num


def check_sum(data):
    sum = 0

    for i in range(41):
        sum = sum + int(data[i])

    a = sum // 256
    b = sum % 256

    check_sum_1 = (a == data[41])
    check_sum_2 = (b == data[42])
    return check_sum_1 and check_sum_2


def uint32_to_float(value_str):
    value = int(value_str, 16)
    return float.fromhex("0x{:x}".format(value))


def hex_to_int(value_str):
    return int(value_str, 16)


def digital_to_voltage(digital_code):
    return 5.0 * digital_code / 65535.0


# Function for: Acceleration, Angular Rate & Magnetometer Vector
def opt1():
    global sample_time_ref
    blist = [203]
    arr = bytes(blist)
    ser.write(arr)
    data = []
    a = ser.read(43)
    # %%
    for k in range(43):
        data.append(a[k])

    while sample_time_ref is None:
        global start_time
        start_time = time.time()
        sample_time_ref = uint32_to_float(concatenate_hex_bytes(data)[74:82])

    print("———— %s seconds ————" % (time.time() - start_time),flush=True)
    sample_time = (uint32_to_float(concatenate_hex_bytes(data)[74:82]) - sample_time_ref) / 19660800

    accel_x = hex_to_float(concatenate_hex_bytes(data)[2:10])
    accel_y = hex_to_float(concatenate_hex_bytes(data)[10:18])
    accel_z = hex_to_float(concatenate_hex_bytes(data)[18:26])

    ang_x = hex_to_float(concatenate_hex_bytes(data)[26:34])
    ang_y = hex_to_float(concatenate_hex_bytes(data)[34:42])
    ang_z = hex_to_float(concatenate_hex_bytes(data)[42:50])

    mag_x = hex_to_float(concatenate_hex_bytes(data)[50:58])
    mag_y = hex_to_float(concatenate_hex_bytes(data)[58:66])
    mag_z = hex_to_float(concatenate_hex_bytes(data)[66:74])

    acc_magnitude = math.sqrt(accel_x ** 2 + accel_y ** 2 + accel_z ** 2)
    ang_magnitude = math.sqrt(ang_x ** 2 + ang_y ** 2 + ang_z ** 2)

    time_val.append(sample_time)

    acc_x_val.append(accel_x)
    acc_y_val.append(accel_y)
    acc_z_val.append(accel_z)

    print("Acceleration x:" + str(accel_x),flush=True)
    print("Acceleration y:" + str(accel_y),flush=True)
    print("Acceleration z:" + str(accel_z),flush=True)

    ang_x_val.append(ang_x)
    ang_y_val.append(ang_y)
    ang_z_val.append(ang_z)

    mag_x_val.append(mag_x)
    mag_y_val.append(mag_y)
    mag_z_val.append(mag_z)

    acc_magnitude_val.append(acc_magnitude)
    ang_magnitude_val.append(ang_magnitude)

    with open("data.txt", "a") as file:
        file.write(f"accel_x = {accel_x}" + "\n" + f"accel_y = {accel_y}" + "\n"
                   + f"accel_z = {accel_z}" + "\n" + f"ang_x = {ang_x}" + "\n"
                   + f"ang_y = {ang_y}" + "\n" + f"ang_z = {ang_z}" + "\n"
                   + f"mag_x = {mag_x}" + "\n" + f"mag_y = {mag_y}" + "\n"
                   + f"mag_z = {mag_z}" + "\n" + "There is no error: " + str(check_sum(data))
                   + "\n" + f"time = {sample_time}" + "\n")

    # -0.993787407875
    if mag_z < -0.05:
        print("The door is opened",flush=True)
        GPIO.output(12, GPIO.HIGH)
        # IoT Mail via IFTTT
        # requests.post('https://maker.ifttt.com/trigger/deneme/with/key/dJ_cSEiASEc1B_B2YxXsOI')
    elif mag_z >= 0.55:
        print("The door is closed",flush=True)
        GPIO.output(12, GPIO.LOW)
    time.sleep(0.00000000000000000001)


# Function for: Temperatures
def opt2():
    global sample_time_ref
    blist = [209]
    arr = bytes(blist)
    ser.write(arr)
    data = []
    a = ser.read(15)
    # %%
    for k in range(15):
        data.append(a[k])

    while sample_time_ref is None:
        global start_time
        start_time = time.time()
        sample_time_ref = uint32_to_float(concatenate_hex_bytes(data)[18:26])

    print("--- %s seconds ---" % (time.time() - start_time),flush=True)
    sample_time = (uint32_to_float(concatenate_hex_bytes(data)[18:26]) - sample_time_ref) / 19660800

    temp_accel = (hex_to_int(concatenate_hex_bytes(data)[2:6]) * (3.3 / 4096) - 0.5) * 100
    temp_gyro_x = (hex_to_int(concatenate_hex_bytes(data)[6:10]) * (3.3 / 4096) - 2.5) * (1 / 0.009) + 25
    temp_gyro_y = (hex_to_int(concatenate_hex_bytes(data)[10:14]) * (3.3 / 4096) - 2.5) * (1 / 0.009) + 25
    temp_gyro_z = (hex_to_int(concatenate_hex_bytes(data)[14:18]) * (3.3 / 4096) - 2.5) * (1 / 0.009) + 25

    time_val.append(sample_time)

    temp_accel_val.append(temp_accel)
    temp_gyro_x_val.append(temp_gyro_x)
    temp_gyro_y_val.append(temp_gyro_y)
    temp_gyro_z_val.append(temp_gyro_z)

    time.sleep(0.01)


# Function for: Raw Accelerometer and Angular Rate Sensor Outputs
def opt3():
    global sample_time_ref
    blist = [193]
    arr = bytes(blist)
    ser.write(arr)
    data = []
    a = ser.read(31)
    # %%
    for k in range(31):
        data.append(a[k])

    while sample_time_ref is None:
        global start_time
        start_time = time.time()
        sample_time_ref = uint32_to_float(concatenate_hex_bytes(data)[50:58])

    print("--- %s seconds ---" % (time.time() - start_time),flush=True)
    sample_time = (uint32_to_float(concatenate_hex_bytes(data)[50:58]) - sample_time_ref) / 19660800

    raw_accel_x = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[2:10]))
    raw_accel_y = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[10:18]))
    raw_accel_z = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[18:26]))

    raw_ang_x = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[26:34]))
    raw_ang_y = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[34:42]))
    raw_ang_z = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[42:50]))

    time_val.append(sample_time)

    raw_acc_x_val.append(raw_accel_x)
    raw_acc_y_val.append(raw_accel_y)
    raw_acc_z_val.append(raw_accel_z)

    raw_ang_x_val.append(raw_ang_x)
    raw_ang_y_val.append(raw_ang_y)
    raw_ang_z_val.append(raw_ang_z)

    time.sleep(0.01)


# Function for: Gyro Stabilized Acceleration, Angular Rate & Magnetometer Vector
def opt4():
    global sample_time_ref
    blist = [210]
    arr = bytes(blist)
    ser.write(arr)
    data = []
    a = ser.read(43)
    # %%
    for k in range(43):
        data.append(a[k])

    while sample_time_ref is None:
        global start_time
        start_time = time.time()
        sample_time_ref = uint32_to_float(concatenate_hex_bytes(data)[74:82])

    print("--- %s seconds ---" % (time.time() - start_time),flush=True)
    sample_time = (uint32_to_float(concatenate_hex_bytes(data)[74:82]) - sample_time_ref) / 19660800

    stab_accel_x = hex_to_float(concatenate_hex_bytes(data)[2:10])
    stab_accel_y = hex_to_float(concatenate_hex_bytes(data)[10:18])
    stab_accel_z = hex_to_float(concatenate_hex_bytes(data)[18:26])

    ang_x = hex_to_float(concatenate_hex_bytes(data)[26:34])
    ang_y = hex_to_float(concatenate_hex_bytes(data)[34:42])
    ang_z = hex_to_float(concatenate_hex_bytes(data)[42:50])

    stab_mag_x = hex_to_float(concatenate_hex_bytes(data)[50:58])
    stab_mag_y = hex_to_float(concatenate_hex_bytes(data)[58:66])
    stab_mag_z = hex_to_float(concatenate_hex_bytes(data)[66:74])

    time_val.append(sample_time)

    stab_acc_x_val.append(stab_accel_x)
    stab_acc_y_val.append(stab_accel_y)
    stab_acc_z_val.append(stab_accel_z)

    ang_x_val.append(ang_x)
    ang_y_val.append(ang_y)
    ang_z_val.append(ang_z)

    stab_mag_x_val.append(stab_mag_x)
    stab_mag_y_val.append(stab_mag_y)
    stab_mag_z_val.append(stab_mag_z)

    time.sleep(0.01)


# Function for: DeltaAngle & DeltaVelocity & Magnetometer Vectors
def opt5():
    global sample_time_ref
    blist = [211]
    arr = bytes(blist)
    ser.write(arr)
    data = []
    a = ser.read(43)
    # %%
    for k in range(43):
        data.append(a[k])

    while sample_time_ref is None:
        global start_time
        start_time = time.time()
        sample_time_ref = uint32_to_float(concatenate_hex_bytes(data)[74:82])

    print("--- %s seconds ---" % (time.time() - start_time),flush=True)
    sample_time = (uint32_to_float(concatenate_hex_bytes(data)[74:82]) - sample_time_ref) / 19660800

    del_accel_x = hex_to_float(concatenate_hex_bytes(data)[2:10])
    del_accel_y = hex_to_float(concatenate_hex_bytes(data)[10:18])
    del_accel_z = hex_to_float(concatenate_hex_bytes(data)[18:26])

    del_ang_x = hex_to_float(concatenate_hex_bytes(data)[26:34])
    del_ang_y = hex_to_float(concatenate_hex_bytes(data)[34:42])
    del_ang_z = hex_to_float(concatenate_hex_bytes(data)[42:50])

    mag_x = hex_to_float(concatenate_hex_bytes(data)[50:58])
    mag_y = hex_to_float(concatenate_hex_bytes(data)[58:66])
    mag_z = hex_to_float(concatenate_hex_bytes(data)[66:74])

    time_val.append(sample_time)

    del_acc_x_val.append(del_accel_x)
    del_acc_y_val.append(del_accel_y)
    del_acc_z_val.append(del_accel_z)

    del_ang_x_val.append(del_ang_x)
    del_ang_y_val.append(del_ang_y)
    del_ang_z_val.append(del_ang_z)

    mag_x_val.append(mag_x)
    mag_y_val.append(mag_y)
    mag_z_val.append(mag_z)

    time.sleep(0.01)


# Function for: Euler Angles and Angular Rates
def opt6():
    global sample_time_ref
    blist = [207]
    arr = bytes(blist)
    ser.write(arr)
    data = []
    a = ser.read(31)
    # %%
    for k in range(31):
        data.append(a[k])

    while sample_time_ref is None:
        global start_time
        start_time = time.time()
        sample_time_ref = uint32_to_float(concatenate_hex_bytes(data)[50:58])

    print("--- %s seconds ---" % (time.time() - start_time),flush=True)
    sample_time = (uint32_to_float(concatenate_hex_bytes(data)[50:58]) - sample_time_ref) / 19660800

    euler_roll = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[2:10]))
    euler_pitch = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[10:18]))
    euler_yaw = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[18:26]))

    ang_x = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[26:34]))
    ang_y = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[34:42]))
    ang_z = digital_to_voltage(hex_to_float(concatenate_hex_bytes(data)[42:50]))

    time_val.append(sample_time)

    euler_roll_val.append(euler_roll)
    euler_pitch_val.append(euler_pitch)
    euler_yaw_val.append(euler_yaw)

    ang_x_val.append(ang_x)
    ang_y_val.append(ang_y)
    ang_z_val.append(ang_z)

    time.sleep(0.01)


# Function for: Orientation Matrix
def opt7():
    global sample_time_ref
    blist = [197]
    arr = bytes(blist)
    ser.write(arr)
    data = []
    a = ser.read(43)
    # %%
    for k in range(43):
        data.append(a[k])

    while sample_time_ref is None:
        global start_time
        start_time = time.time()
        sample_time_ref = uint32_to_float(concatenate_hex_bytes(data)[74:82])

    print("--- %s seconds ---" % (time.time() - start_time),flush=True)
    sample_time = (uint32_to_float(concatenate_hex_bytes(data)[74:82]) - sample_time_ref) / 19660800

    M1_1 = hex_to_float(concatenate_hex_bytes(data)[2:10])
    M1_2 = hex_to_float(concatenate_hex_bytes(data)[10:18])
    M1_3 = hex_to_float(concatenate_hex_bytes(data)[18:26])

    M2_1 = hex_to_float(concatenate_hex_bytes(data)[26:34])
    M2_2 = hex_to_float(concatenate_hex_bytes(data)[34:42])
    M2_3 = hex_to_float(concatenate_hex_bytes(data)[42:50])

    M3_1 = hex_to_float(concatenate_hex_bytes(data)[50:58])
    M3_2 = hex_to_float(concatenate_hex_bytes(data)[58:66])
    M3_3 = hex_to_float(concatenate_hex_bytes(data)[66:74])

    time_val.append(sample_time)

    M1_1_val.append(M1_1)
    M1_2_val.append(M1_2)
    M1_3_val.append(M1_3)

    M2_1_val.append(M2_1)
    M2_2_val.append(M2_2)
    M2_3_val.append(M2_3)

    M3_1_val.append(M3_1)
    M3_2_val.append(M3_2)
    M3_3_val.append(M3_3)

    global M
    M = np.array([[M1_1_val[-1], M1_2_val[-1], M1_3_val[-1]], [M2_1_val[-1], M2_2_val[-1], M2_3_val[-1]],
                  [M3_1_val[-1], M3_2_val[-1], M3_3_val[-1]]])

    time.sleep(0.01)


# Function for: Attitude Update Matrix
def opt8():
    global sample_time_ref
    blist = [198]
    arr = bytes(blist)
    ser.write(arr)
    data = []
    a = ser.read(43)
    # %%
    for k in range(43):
        data.append(a[k])

    while sample_time_ref is None:
        global start_time
        start_time = time.time()
        sample_time_ref = uint32_to_float(concatenate_hex_bytes(data)[74:82])

    print("--- %s seconds ---" % (time.time() - start_time),flush=True)
    sample_time = (uint32_to_float(concatenate_hex_bytes(data)[74:82]) - sample_time_ref) / 19660800

    C1_1 = hex_to_float(concatenate_hex_bytes(data)[2:10])
    C1_2 = hex_to_float(concatenate_hex_bytes(data)[10:18])
    C1_3 = hex_to_float(concatenate_hex_bytes(data)[18:26])

    C2_1 = hex_to_float(concatenate_hex_bytes(data)[26:34])
    C2_2 = hex_to_float(concatenate_hex_bytes(data)[34:42])
    C2_3 = hex_to_float(concatenate_hex_bytes(data)[42:50])

    C3_1 = hex_to_float(concatenate_hex_bytes(data)[50:58])
    C3_2 = hex_to_float(concatenate_hex_bytes(data)[58:66])
    C3_3 = hex_to_float(concatenate_hex_bytes(data)[66:74])

    time_val.append(sample_time)

    C1_1_val.append(C1_1)
    C1_2_val.append(C1_2)
    C1_3_val.append(C1_3)

    C2_1_val.append(C2_1)
    C2_2_val.append(C2_2)
    C2_3_val.append(C2_3)

    C3_1_val.append(C3_1)
    C3_2_val.append(C3_2)
    C3_3_val.append(C3_3)

    global C
    C = np.array([[C1_1_val[-1], C1_2_val[-1], C1_3_val[-1]], [C2_1_val[-1], C2_2_val[-1], C2_3_val[-1]],
                  [C3_1_val[-1], C3_2_val[-1], C3_3_val[-1]]])

    time.sleep(0.01)


def start_loop():
    global start_flag
    start_flag = True
    global running
    running = True
    while running:
        if choice == "opt1":
            opt1()
        elif choice == "opt2":
            opt2()
        elif choice == "opt3":
            opt3()
        elif choice == "opt4":
            opt4()
        elif choice == "opt5":
            opt5()
        elif choice == "opt6":
            opt6()
        elif choice == "opt7":
            opt7()
        elif choice == "opt8":
            opt8()


def low_pass_filter(data, alpha):
    # Initialize filtered data array with the first value from the input data
    filtered_data = np.zeros(data.shape)
    filtered_data[0] = data[0]

    # Apply the low - pass filter to each subsequent value in the data array
    for i in range(1, data.shape[0]):
        filtered_data[i] = (1 - alpha) * filtered_data[i - 1] + alpha * data[i]

    return filtered_data


def plot_data(signum, frame):
    if choice == "opt1":
        fig1 = plt.figure()
        fig1.suptitle("Original Data", fontsize=16)
        plt.subplot(3, 1, 1)
        plt.plot(time_val, acc_x_val, color="blue")
        plt.plot(time_val, acc_y_val, color="red")
        plt.plot(time_val, acc_z_val)
        plt.plot(time_val, acc_magnitude_val, color="magenta")
        plt.ylabel("Linear Acceleration \n (g = 9.80665 m/s$^2$)")
        plt.legend(["x axis", "y axis", "z axis", "magnitude"])
        plt.subplot(3, 1, 2)
        plt.plot(time_val, ang_x_val, color="blue")
        plt.plot(time_val, ang_y_val, color="red")
        plt.plot(time_val, ang_z_val)
        plt.plot(time_val, ang_magnitude_val, color="magenta")
        plt.ylabel("Angular Velocity (rad/s)")
        plt.legend(["x axis", "y axis", "z axis", "magnitude"])
        plt.subplot(3, 1, 3)
        plt.plot(time_val, mag_x_val, color="blue")
        plt.plot(time_val, mag_y_val, color="red")
        plt.plot(time_val, mag_z_val)
        plt.xlabel("Time (s)")
        plt.ylabel("Instantaneous Magnetometer \n Direction and Magnitude (Gauss)")
        plt.legend(["x axis", "y axis", "z axis"])

        fig2 = plt.figure()
        fig2.suptitle("Filtered (LPF) Data", fontsize=16)
        # Create numpy arrays for later using variable lists as arrays
        np_acc_x = np.array(acc_x_val)
        np_acc_y = np.array(acc_y_val)
        np_acc_z = np.array(acc_z_val)

        np_ang_x = np.array(ang_x_val)
        np_ang_y = np.array(ang_y_val)
        np_ang_z = np.array(ang_z_val)

        np_acc_magnitude = np.array(acc_magnitude_val)
        np_ang_magnitude = np.array(ang_magnitude_val)
        # Test the low - pass filter with some synthetic data
        sample_rate = 100  # 100Hz according to the data analysis and Data Communications Protocol
        alpha = 0.5  # Filter coefficient

        # Filter the data

        # Filtered linear acceleration data
        filtered_acc_x = low_pass_filter(np_acc_x, alpha)
        filtered_acc_y = low_pass_filter(np_acc_y, alpha)
        filtered_acc_z = low_pass_filter(np_acc_z, alpha)

        # Filtered angular velocity data
        filtered_ang_x = low_pass_filter(np_ang_x, alpha)
        filtered_ang_y = low_pass_filter(np_ang_y, alpha)
        filtered_ang_z = low_pass_filter(np_ang_z, alpha)

        # Filtered angular velocity and acceleration magnitudes
        filtered_acc_magnitude = low_pass_filter(np_acc_magnitude, alpha)
        filtered_ang_magnitude = low_pass_filter(np_ang_magnitude, alpha)

        plt.subplot(2, 1, 1)
        plt.plot(time_val, filtered_acc_x, color="blue")
        plt.plot(time_val, filtered_acc_y, color="red")
        plt.plot(time_val, filtered_acc_z)
        plt.plot(time_val, filtered_acc_magnitude, color="magenta")
        plt.ylabel("Linear Acceleration (g = 9.80665 m/s$^2$)")
        plt.legend(["x axis", "y axis", "z axis", "magnitude"])
        plt.subplot(2, 1, 2)
        plt.plot(time_val, filtered_ang_x, color="blue")
        plt.plot(time_val, filtered_ang_y, color="red")
        plt.plot(time_val, filtered_ang_z)
        plt.plot(time_val, filtered_ang_magnitude, color="magenta")
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (rad/s)")
        plt.legend(["x axis", "y axis", "z axis", "magnitude"])

    elif choice == "opt2":
        fig1 = plt.figure()
        fig1.suptitle("Temperatures According to Differently Located Temperature Sensors", fontsize=16)
        plt.plot(time_val, temp_accel_val, color="blue")
        plt.plot(time_val, temp_gyro_x_val, color="red")
        plt.plot(time_val, temp_gyro_y_val)
        plt.plot(time_val, temp_gyro_z_val)
        plt.xlabel("Time (s)")
        plt.ylabel("Temperature \N{DEGREE SIGN}C")
        plt.legend(["accelerometer sensors", "X gyro sensor", "Y gyro sensor", "Z gyro sensor"])

    elif choice == "opt3":
        fig1 = plt.figure()
        fig1.suptitle("Raw Voltage Outputs", fontsize=16)
        plt.subplot(2, 1, 1)
        plt.plot(time_val, raw_acc_x_val, color="blue")
        plt.plot(time_val, raw_acc_y_val, color="red")
        plt.plot(time_val, raw_acc_z_val)
        plt.ylabel("Raw Voltage Outputs (V)")
        plt.legend(["RawAccel_1", "RawAccel_2", "RawAccel_2"])
        plt.subplot(2, 1, 2)
        plt.plot(time_val, raw_ang_x_val, color="blue")
        plt.plot(time_val, raw_ang_y_val, color="red")
        plt.plot(time_val, raw_ang_z_val)
        plt.xlabel("Time (s)")
        plt.ylabel("Raw Voltage Outputs (V)")
        plt.legend(["RawAngRate1", "RawAngRate2", "RawAngRate3"])

    elif choice == "opt4":
        fig1 = plt.figure()
        fig1.suptitle("Original Data", fontsize=16)
        plt.subplot(3, 1, 1)
        plt.plot(time_val, stab_acc_x_val, color="blue")
        plt.plot(time_val, stab_acc_y_val, color="red")
        plt.plot(time_val, stab_acc_z_val)
        plt.ylabel("Gyro Stabilized Acceleration \n (g = 9.80665 m/s$^2$)")
        plt.legend(["x axis", "y axis", "z axis"])
        plt.subplot(3, 1, 2)
        plt.plot(time_val, ang_x_val, color="blue")
        plt.plot(time_val, ang_y_val, color="red")
        plt.plot(time_val, ang_z_val)
        plt.ylabel("Angular Velocity (rad/s)")
        plt.legend(["x axis", "y axis", "z axis"])
        plt.subplot(3, 1, 3)
        plt.plot(time_val, stab_mag_x_val, color="blue")
        plt.plot(time_val, stab_mag_y_val, color="red")
        plt.plot(time_val, stab_mag_z_val)
        plt.xlabel("Time (s)")
        plt.ylabel("Gyro Stabilized Magnetometer Vector (Gauss)")
        plt.legend(["x axis", "y axis", "z axis"])

        fig2 = plt.figure()
        fig2.suptitle("Filtered (LPF) Data", fontsize=16)
        # Create numpy arrays for later using variable lists as arrays
        np_stab_acc_x = np.array(stab_acc_x_val)
        np_stab_acc_y = np.array(stab_acc_y_val)
        np_stab_acc_z = np.array(stab_acc_z_val)

        np_ang_x = np.array(ang_x_val)
        np_ang_y = np.array(ang_y_val)
        np_ang_z = np.array(ang_z_val)
        # Test the low - pass filter with some synthetic data
        sample_rate = 100  # 100Hz according to the data analysis and Data Communications Protocol
        alpha = 0.5  # Filter coefficient

        # Filter the data

        # Filtered linear acceleration data
        filtered_stab_acc_x = low_pass_filter(np_stab_acc_x, alpha)
        filtered_stab_acc_y = low_pass_filter(np_stab_acc_y, alpha)
        filtered_stab_acc_z = low_pass_filter(np_stab_acc_z, alpha)

        # Filtered angular velocity data
        filtered_ang_x = low_pass_filter(np_ang_x, alpha)
        filtered_ang_y = low_pass_filter(np_ang_y, alpha)
        filtered_ang_z = low_pass_filter(np_ang_z, alpha)

        plt.subplot(2, 1, 1)
        plt.plot(time_val, filtered_stab_acc_x, color="blue")
        plt.plot(time_val, filtered_stab_acc_y, color="red")
        plt.plot(time_val, filtered_stab_acc_z)
        plt.ylabel("Gyro Stabilized Acceleration (g = 9.80665 m/s$^2$)")
        plt.legend(["x axis", "y axis", "z axis"])
        plt.subplot(2, 1, 2)
        plt.plot(time_val, filtered_ang_x, color="blue")
        plt.plot(time_val, filtered_ang_y, color="red")
        plt.plot(time_val, filtered_ang_z)
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (rad/s)")
        plt.legend(["x axis", "y axis", "z axis"])

    elif choice == "opt5":
        fig1 = plt.figure()
        fig1.suptitle("DeltaAngle & DeltaVelocity & Magnetometer Vectors", fontsize=16)
        plt.subplot(3, 1, 1)
        plt.plot(time_val, del_acc_x_val, color="blue")
        plt.plot(time_val, del_acc_y_val, color="red")
        plt.plot(time_val, del_acc_z_val)
        plt.ylabel("Time Integral of Acceleration (g$^*$)")
        plt.legend(["x axis", "y axis", "z axis"])
        plt.subplot(3, 1, 2)
        plt.plot(time_val, del_ang_x_val, color="blue")
        plt.plot(time_val, del_ang_y_val, color="red")
        plt.plot(time_val, del_ang_z_val)
        plt.ylabel("Time Integral of Angular Velocity (rad)")
        plt.legend(["x axis", "y axis", "z axis"])
        plt.subplot(3, 1, 3)
        plt.plot(time_val, mag_x_val, color="blue")
        plt.plot(time_val, mag_y_val, color="red")
        plt.plot(time_val, mag_z_val)
        plt.xlabel("Time (s)")
        plt.ylabel("Magnetometer Vector (Gauss)")
        plt.legend(["x axis", "y axis", "z axis"])

    elif choice == "opt6":
        fig1 = plt.figure()
        fig1.suptitle("Euler Angles and Angular Rates", fontsize=16)
        plt.subplot(2, 1, 1)
        plt.plot(time_val, euler_roll_val, color="blue")
        plt.plot(time_val, euler_pitch_val, color="red")
        plt.plot(time_val, euler_yaw_val)
        plt.ylabel(" Roll, Pitch and Yaw Angles (rad)")
        plt.legend(["roll", "pitch", "yaw"])
        plt.subplot(2, 1, 2)
        plt.plot(time_val, ang_x_val, color="blue")
        plt.plot(time_val, ang_y_val, color="red")
        plt.plot(time_val, ang_z_val)
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Velocity (rad/s)")
        plt.legend(["x axis", "y axis", "z axis"])

    elif choice == "opt7":
        global M
        # Flatten the 3D array to 2D
        M = M.reshape(-1, M.shape[-1])
        print("M = ", M,flush=True)
        # Display the matrix in a figure
        fig, ax = plt.subplots()
        im = ax.imshow(M, cmap=plt.cm.Blues)
        fig.colorbar(im)
        plt.title("9 Component Coordinate Transformation Matrix "
                  "which describes the Orientation", fontsize=16)
        # Display the values of the elements
        for i in range(M.shape[0]):
            for j in range(M.shape[1]):
                ax.text(j, i, "{:.2f}".format(M[i, j]), ha="center", va="center")

    elif choice == "opt8":
        global C
        # Flatten the 3D array to 2D
        C = C.reshape(-1, C.shape[-1])
        print("C = ", C,flush=True)
        # Display the matrix in a figure
        fig, ax = plt.subplots()
        im = ax.imshow(C, cmap=plt.cm.Blues)
        fig.colorbar(im)
        plt.title("9 Component Coordinate Transformation Matrix "
                  "which describes the Change in Orientation", fontsize=16)
        # Display the values of the elements
        for i in range(C.shape[0]):
            for j in range(C.shape[1]):
                ax.text(j, i, "{:.2f}".format(C[i, j]), ha="center", va="center")

    plt.show()
    exit(1)

# Changing behavior of stopping the code
signal.signal(signal.SIGINT, plot_data)
start_loop()