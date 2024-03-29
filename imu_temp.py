
#!/usr/bin/env python


import smbus            #import SMBus module of I2C
import time
import array as arr

import statistics as st
import math as m


PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

pi = 3.1415926
to_deg = 180/pi
to_rad = pi/180

gyro_rate_adjust = 1 # not used
acc_rate_adjust = 1  # not used

alpha=0.95
acc_ref = 16834.0
gyro_ref = 131.0

gyro_angle=[0,0,0]
prev_acc = [0,0,0]
combined_angle = [0,0,0]

gyro_offset_x,gyro_offset_y,gyro_offset_z=0,0,0


def offset_calculation():
    gyro_x_check,gyro_y_check,gyro_z_check = [],[],[]
    global gyro_offset_x,gyro_offset_y,gyro_offset_z
    
    #looping to find no. of samples
    for i in range(1000):
        gyro_x_check.append(read_raw_data(GYRO_XOUT_H))
        gyro_y_check.append(read_raw_data(GYRO_YOUT_H))
        gyro_z_check.append(read_raw_data(GYRO_ZOUT_H))
    
    #offset calculation through mean of 1000 samples
    gyro_offset_x =st.mean(gyro_x_check)
    gyro_offset_y =st.mean(gyro_y_check)
    gyro_offset_z =st.mean(gyro_z_check)


def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 0)
    
    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)


def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value


def euler_to_quaternion(yaw, pitch, roll):
    qx = m.sin(roll/2) * m.cos(pitch/2) * m.cos(yaw/2) - m.cos(roll/2) * m.sin(pitch/2) * m.sin(yaw/2)
    qy = m.cos(roll/2) * m.sin(pitch/2) * m.cos(yaw/2) + m.sin(roll/2) * m.cos(pitch/2) * m.sin(yaw/2)
    qz = m.cos(roll/2) * m.cos(pitch/2) * m.sin(yaw/2) - m.sin(roll/2) * m.sin(pitch/2) * m.cos(yaw/2)
    qw = m.cos(roll/2) * m.cos(pitch/2) * m.cos(yaw/2) + m.sin(roll/2) * m.sin(pitch/2) * m.sin(yaw/2)

    return qx, qy, qz, qw


def read_raw_values():
    global gyro_offset_x,gyro_offset_y,gyro_offset_z
    #read accelerometer raw value
    acc_x_raw = read_raw_data(ACCEL_XOUT_H)
    acc_y_raw = read_raw_data(ACCEL_YOUT_H)
    acc_z_raw = read_raw_data(ACCEL_ZOUT_H)
    #read accelerometer raw value
    gyro_x=read_raw_data(GYRO_XOUT_H) - gyro_offset_x
    gyro_y=read_raw_data(GYRO_YOUT_H) - gyro_offset_y
    gyro_z=read_raw_data(GYRO_ZOUT_H) - gyro_offset_z

    return [acc_x_raw, acc_y_raw, acc_z_raw], [gyro_x, gyro_y, gyro_z]


def normalize_acc_data(acc_raw):
    global acc_ref
    #normalize acc values
    a_norm_x = acc_raw[0]/acc_ref
    a_norm_y = acc_raw[1]/acc_ref
    a_norm_z = acc_raw[2]/acc_ref
    
    return [a_norm_x, a_norm_y, a_norm_z]


def normalize_gyro_data(gyro_raw):
    global gyro_ref
    #normalize gyro values
    g_norm_x = gyro_raw[0]/gyro_ref
    g_norm_y = gyro_raw[1]/gyro_ref
    g_norm_z = gyro_raw[2]/gyro_ref

    return [g_norm_x, g_norm_y, g_norm_z]


def get_angle_gyro(gyro_angle, gyro, dt):
    #determine angle due to gyro effect
    gyro_angle_x = gyro_angle[0] + gyro[0]*dt
    gyro_angle_y = gyro_angle[1] + gyro[1]*dt
    gyro_angle_z = gyro_angle[2] + gyro[2]*dt
    
    return [gyro_angle_x,gyro_angle_y,gyro_angle_z]


def get_median(acc_x, acc_y, acc_z):
    a_x=st.median(acc_x)
    a_y=st.median(acc_y)
    a_z=st.median(acc_z)
    
    return [a_x, a_y, a_z]


def get_angle_acc(acc):
    try:
        #acc_angle_x = m.atan2(acc[1], acc[2]) * to_deg
        #acc_angle_y = m.atan2((-1*acc[0]), m.sqrt(acc[1]**2 + acc[2]**2)) * to_deg
        #acc_angle_z = m.acos(acc[2] / m.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)) * to_deg
        acc_angle_x = m.asin(acc[1] / m.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)) * to_deg
        acc_angle_y = m.asin(-acc[0] / m.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)) * to_deg
        acc_angle_z = m.acos(acc[2] / m.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)) * to_deg
    except Exception as e:
        pass
    
    return [acc_angle_x, acc_angle_y, acc_angle_z]


def get_alpha(prev_acc, acc):
    # Complementary filter is applied on basis of rate of accelrometer measurement
    alpha_x = abs((prev_acc[0] - acc[0]))
    alpha_y = abs((prev_acc[1] - acc[1]))
    alpha_z = abs((prev_acc[2] - acc[2]))
    alpha = max(alpha_x, alpha_y, alpha_z)
    alpha = alpha * 10
    if alpha > 1:
        alpha = 1
        
    return alpha


def get_combined_angle(acc_angle, gyro_angle, alpha):
    print(alpha)
    combined_angle_x = alpha*gyro_angle[0] + (1-alpha)*acc_angle[0]
    combined_angle_y = alpha*gyro_angle[1] + (1-alpha)*acc_angle[1]
    combined_angle_z = gyro_angle[2]
    
    return [combined_angle_x, combined_angle_y, combined_angle_z]


def get_velocity(vel_prev, acc_norm, dt):
    # accc in m/s2
    #acc_actual = [x * (9.8) for x in acc_norm]
    acc_actual = [acc_norm[0] * 9.8, acc_norm[1] * 9.8, acc_norm[2] * 9.8]
    vel_x = vel_prev[0] + acc_actual[0]*dt
    vel_y = vel_prev[1] + acc_actual[1]*dt
    vel_z = vel_prev[2] + acc_actual[2]*dt
    
    return [vel_x, vel_y, vel_z]

def get_position(prev_position, prev_velocity, dt):
    # accc in m/s2
    pos_x = prev_position[0] + prev_velocity[0]*dt
    pos_y = prev_position[1] + prev_velocity[1]*dt
    pos_z = prev_position[2] + prev_velocity[2]*dt
    
    return [pos_x, pos_y, pos_z]


def gravity_compensation(acc_norm, combined_angle):
    x_acc_linear = acc_norm[0] + m.sin(combined_angle[1] * to_rad);      #16384 height latitude compensation
    y_acc_linear = acc_norm[1] - m.sin(combined_angle[0] * to_rad);      #16384
    z_acc_linear = acc_norm[2] - m.cos(combined_angle[2] * to_rad);      #16384
    
    return [x_acc_linear, y_acc_linear, z_acc_linear]


prev_velocity = [0,0,0]
prev_velocity_raw = [0,0,0]
prev_position = [0,0,0]
acc_accumulation = []
count = 0
no_of_acc_samples = 10
def get_data(dt):
    global gyro_angle, prev_acc, prev_velocity, prev_position, combined_angle
    global alpha, prev_velocity_raw, count, acc_accumulation, no_of_acc_samples
    
    gyro_angle = combined_angle
    acc_raw, gyro_raw = read_raw_values()
    
    gyro_norm = normalize_gyro_data(gyro_raw)
    acc_norm = normalize_acc_data(acc_raw)
    
    acc_angle = get_angle_acc(acc_norm)
    print("Accelerometer Angle:")
    print("\tx: ",acc_angle[0])
    print("\ty: ",acc_angle[1])
    print("\tz: ",acc_angle[2])

    gyro_angle = get_angle_gyro(gyro_angle, gyro_norm, dt)
    #print("gyro_angle",gyro_angle)
    print("Gyroscope Angle:")
    print("\tx: ",gyro_angle[0])
    print("\ty: ",gyro_angle[1])
    print("\tz: ",gyro_angle[2])

    velocity_raw = get_velocity(prev_velocity_raw, acc_norm, dt)

    #alpha = get_alpha(prev_acc, acc_norm)
    #alpha = get_alpha(prev_velocity_raw, velocity_raw)
    print("Alpha: ",alpha)
    
    combined_angle = get_combined_angle(acc_angle, gyro_angle, alpha)
    #print("Combined angle: ",combined_angle)
    
    angle_for_compensation = [combined_angle[0], combined_angle[1], acc_angle[2]]
    #print(angle_for_compensation)
    print("Angle for Compensation:")
    print("\tx: ",angle_for_compensation[0])
    print("\ty: ",angle_for_compensation[1])
    print("\tz: ",angle_for_compensation[2])

    acc_norm_compensated = gravity_compensation(acc_norm, angle_for_compensation)
    #print("Raw",acc_norm)
    #print("Compensated",acc_norm_compensated)
    print("Accelerometer Norm:")
    print("\tx: ",acc_norm[0])
    print("\ty: ",acc_norm[1])
    print("\tz: ",acc_norm[2])

    print("Compensated Accelerometer Normal:")
    print("\tx: ",acc_norm_compensated[0])
    print("\ty: ",acc_norm_compensated[1])
    print("\tz: ",acc_norm_compensated[2])
    
    velocity = get_velocity(prev_velocity, acc_norm_compensated, dt)
    #print("Velocity: ",velocity)
    print("Velocity:")
    print("\tx: ",velocity[0])
    print("\ty: ",velocity[1])
    print("\tz: ",velocity[2])
    
    position = get_position(prev_position, prev_velocity, dt)
    #print("Position: ",position)
    print("position: ")
    print("\tx: ",position[0])
    print("\ty: ",position[1])
    print("\tz: ",position[2])
    print("")

    prev_velocity_raw = velocity_raw     
    prev_acc = acc_norm
    prev_velocity = velocity
    prev_position = position
    
    #combined_angle_rad = list_to_radian(combined_angle)
    
    x, y, z, w = euler_to_quaternion(combined_angle[0]*to_rad, combined_angle[1]*to_rad, combined_angle[2]*to_rad)


bus = smbus.SMBus(1)    # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

if __name__=="__main__":
    MPU_Init()
    offset_calculation()
    print (" Reading Data of Gyroscope and Accelerometer")
    t1 = time.time()
    
    while True:
        try:
            dt=time.time()-t1
            t1=time.time()
            
            get_data(dt)
            #time.sleep(0.1)
        except Exception as e:
            print(e)
            #pass

