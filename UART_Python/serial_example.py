#!/usr/bin/env python3
import serial

# Open the Serial port
ser = serial.Serial("/dev/ttyUSB0") 
print( "Serial [ %s ] is opened" % ser.name ) # check the serial port
frequency = 100
message = "f {}\r".format(frequency)
status = ser.write(message.encode('utf-8'))
ser.close()
print( "Serial [ %s ] is closed" % ser.name )
