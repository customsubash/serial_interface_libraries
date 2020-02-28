#!/usr/bin/python

import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.mode=0
spi.max_speed_hz = 50000
data=0;
# Split an integer input into a two byte array to send via SPI
def write_pot(input):
    msb = input >> 8
    lsb = input & 0xFF
    spi.xfer([msb, lsb])

packetcounter = 30
# Repeatedly switch a MCP4151 digital pot off then on
while True:
    spi.xfer([0x65])
    data = spi.readbytes(3)
    if data>0:
        print(data)
    #write_pot(0x1FF)
    #time.sleep(0.5)
    #write_pot(0x00)
    #time.sleep(0.5)
    #print("hya")
