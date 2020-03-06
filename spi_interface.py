

#!/usr/bin/python3

import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.no_cs=True
spi.max_speed_hz = 1000000
spi.mode = 0b01
data=0;

# Split an integer input into a two byte array to send via SPI
def write_pot(input):
    msb = input >> 8
    lsb = input & 0xFF
    spi.xfer([msb, lsb])

def get_data_on(addr):
   spi.xfer([addr])
   data=spi.xfer([addr])
   return data

def send_ins(data):
    recv=[None]
    data=ord(data)
    while data!=recv[0]:
        da = spi.xfer([data])
        recv = spi.xfer([data])
        print("Sent Data: ", data)
        print("received data: ",recv)
    return

def send_char(data):
    to_send = ord(data)
    spi.xfer([to_send])
    return

# Repeatedly switch a MCP4151 digital pot off then on
while True:
    data = input("Enter the Instruction: ")
    length = len(data)
    if length==1:
        send_char(data)
