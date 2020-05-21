

import smbus            #import SMBus module of I2C
import time
import array as arr

import statistics as st
from math import asin, acos, atan2, sqrt, pi

class imu6050:
    def __init__(self, device_address):
        self.device_address = device_address
        self.raw_acc = [0,0,0]
        self.raw_gyro = [0,0,0]
        self.acc_norm = [0,0,0]
        self.gyro_norm = [0,0,0]
        self.acc_ref = 16834.0  #default is this, but can be changed in mpu_init()
        self.gyro_ref = 131.0   #default is this, but can be changed in mpu_init()
        self.gyro_offset = [0,0,0]

        self.angle_by_acc = [0,0,0]
        self.angle_by_gyro = [0,0,0]
        self.combined_angle = [0,0,0]
        self.compensated_acc = [0,0,0]
        
        self.velocity = [0,0,0]
        self.position = [0,0,0]

        self.PWR_MGMT_1   = 0x6B
        self.SMPLRT_DIV   = 0x19
        self.CONFIG       = 0x1A
        self.GYRO_CONFIG  = 0x1B
        self.INT_ENABLE   = 0x38
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H  = 0x43

        self.bus = smbus.SMBus(1)   # or bus = smbus.SMBus(0) for older version boards
        self.mpu_init()
        self.offset_calculation()
        

    def read_raw_values(self):
        raw_acc, raw_gyro = [0,0,0], [0,0,0]
        for i in range(3):
            #read accelerometer raw value
            raw_acc[i] = self.read_raw_data(self.ACCEL_XOUT_H + i*2)
            #read accelerometer raw value
            raw_gyro[i] = self.read_raw_data(self.GYRO_XOUT_H + i*2) - self.gyro_offset[i]

        self.raw_acc = raw_acc
        self.raw_gyro = raw_gyro
        
                                             
    def mpu_init(self):
        bus = self.bus
        Device_Address = self.device_address
        #write to sample rate register
        bus.write_byte_data(Device_Address, self.SMPLRT_DIV, 7)
        
        #Write to power management register
        bus.write_byte_data(Device_Address, self.PWR_MGMT_1, 1)
        
        #Write to Configuration register
        bus.write_byte_data(Device_Address, self.CONFIG, 0)
        
        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, self.GYRO_CONFIG, 0)
        self.acc_ref = 16834.0
        self.gyro_ref = 131.0
        
        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, self.INT_ENABLE, 1)


    def offset_calculation(self):
        gyro_x_check,gyro_y_check,gyro_z_check = [],[],[]
        
        #looping to find no. of samples
        for i in range(1000):
            gyro_x_check.append(self.read_raw_data(self.GYRO_XOUT_H))
            gyro_y_check.append(self.read_raw_data(self.GYRO_XOUT_H + 2))
            gyro_z_check.append(self.read_raw_data(self.GYRO_XOUT_H + 4))
        
        #offset calculation through mean of 1000 samples
        gyro_offset_x =st.mean(gyro_x_check)
        gyro_offset_y =st.mean(gyro_y_check)
        gyro_offset_z =st.mean(gyro_z_check)
        
        self.gyro_offset = [gyro_offset_x, gyro_offset_y, gyro_offset_z]


    def read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.device_address, addr)
        low = self.bus.read_byte_data(self.device_address, addr+1)

        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
        return value


    def normalize_acc_data(self):
        acc_norm, gyro_norm = [0,0,0],[0,0,0]
        for i in range(3):
            #normalize acc values
            acc_norm[i] = self.raw_acc[i]/self.acc_ref
            #normalize gyro values
            gyro_norm[i] = self.raw_gyro[i]/self.gyro_ref
            
        self.acc_norm = acc_norm
        self.acc_norm = gyro_norm
        

    def get_angle_gyro(self, dt):
        angle_by_gyro = [0,0,0]
        for i in range(3):
            #determine angle due to gyro effect
            angle_by_gyro[i] = self.angle_by_gyro[i] + self.raw_gyro[i]*dt
        self.angle_by_gyro = angle_by_gyro


    def get_angle_acc(self):
        acc = self.acc_norm
        angle_by_acc = [0,0,0]
        try:
            #mag = sqrt(acc_norm[0]**2 + acc_norm[1]**2 + acc_norm[2]**2)
            
            #angle_by_acc[0] = asin(acc_norm[1] / mag)
            #angle_by_acc[1] = asin(-acc_norm[0] / mag)
            #angle_by_acc[2] = acos(acc_norm[2] / mag)
            angle_by_acc[0] = atan2(acc[1], acc[2])
            angle_by_acc[1] = atan2((-1*acc[0]), sqrt(acc[1]**2 + acc[2]**2))
            angle_by_acc[2] = acos(acc[2] / sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2))
            #acc_angle_x = m.asin(acc[1] / m.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)) * to_deg
            #acc_angle_y = m.asin(-acc[0] / m.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)) * to_deg
            #acc_angle_z = m.acos(acc[2] / m.sqrt(acc[0]**2 + acc[1]**2 + acc[2]**2)) * to_deg
            self.angle_by_acc = angle_by_acc
        except Exception as e:
            pass
        

    def convert_to_degree(self, data):
        temp = [0,0,0]
        for i in range(3):
            temp[i] = data[i] * 180/(2 * pi)
        return temp
        
    

    def update(self, dt):
        self.read_raw_values()
        self.normalize_acc_data()
        self.get_angle_gyro(dt)
        self.get_angle_acc()


        
device_address = 0x68
i = imu6050(device_address)

t1 = time.time()
while(1):
    dt = time.time() - t1
    i.update(dt)
    #print(i.convert_to_degree(i.angle_by_acc))
    print(i.raw_acc)
    
    t1 = time.time()

                
