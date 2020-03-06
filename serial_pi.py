'''
UART communication on Raspberry Pi using Pyhton
http://www.electronicwings.com
'''
import serial
from time import sleep

ser = serial.Serial("/dev/ttyS0", 9600)    #Open port with baud rate
while True:
    to_send = input("enter the character")
    to_send_byte = to_send.encode()
    ser.write(to_send_byte)
    #received_data = ser.read()              #read serial port
    #sleep(0.03)
    #data_left = ser.inWaiting()             #check for remaining byte
    #received_data += ser.read(data_left)
    #print (received_data)                   #print received data
    #ser.write(received_data)                #transmit data serially 
