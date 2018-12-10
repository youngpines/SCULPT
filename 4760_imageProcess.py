#!/usr/bin/env python3
#Importing the basic Image library 
from PIL import Image
import numpy, serial, time

#opening up a serial port to transmit to the Microcontroller
ser = serial.Serial(
        port = "/dev/ttyUSB0",
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits = 1,
        bytesize = serial.EIGHTBITS
        )       #the port can change based on which)

#opening up the target image
im = Image.open('bigc.png')

#displaying the target image for checking function
#im.show()

#converting the image to greyscale and resizing
grey = im.convert('L')
size = 10, 10
grey = grey.resize(size,Image.LANCZOS)

#displaying the final to ensure it worked well
grey.show()

#format is [width][height]
pixels = numpy.asarray(grey, dtype=numpy.uint8)


#cycling through and writing all pixel values to the PIC
count = 0
for i in range(0,size[1]):
    for j in range(0,size[0]):
        
        msg = "p {}\r".format(pixels[i][j])
        ser.write(msg.encode('utf-8'))
        time.sleep(0.1)
        count += 1
        print("Count [ %d ] = " % count)
        print(pixels[i][j])
        print(msg)

msg = "e \r"
ser.write(msg.encode('utf-8'))
ser.close()
time.sleep(2)
print( "Ended" )
