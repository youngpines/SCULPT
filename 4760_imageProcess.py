#Importing the basic Image library 
from PIL import Image
import numpy, serial, time

#opening up a serial port to transmit to the Microcontroller
ser = serial.Serial(
    port='/dev/ttyUSB1',        #the port can change based on which is used
    baudrate=9600,
    parity= serial.PARITY_NONE,
    stopbits=1,
    bytesize=8
)

#opening up the target image
im = Image.open('/Users/Jared/Desktop/blkln.jpg')

#displaying the target image for checking function
im.show()

#converting the image to greyscale and resizing
grey = im.convert('L')
size = 248, 128
grey = grey.resize(size,Image.LANCZOS)

#displaying the final to ensure it worked well
grey.show()

#format is [width][height]
pixels = numpy.asarray(grey, dtype=numpy.uint8)

#cycling through and writing all pixel values to the PIC
for i in range(pixels.size[0]):
    for j in range(pixels.size[1]):
        ser.write(pixels[i][j])


