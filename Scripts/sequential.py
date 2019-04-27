# Importing the basic Image library 
from PIL import Image
import numpy as np
import serial, time

# Opening up a serial port to transmit to the Microcontroller
#ser = serial.Serial(
#    port='/dev/ttyUSB0',        #the port can change based on which is used
#    baudrate=9600,
#    parity= serial.PARITY_NONE,
#    stopbits=1,
#    bytesize=8
#)
#print("Using [ " + ser.name + " ]")

# Opening up the target image
im = Image.open('/home/ho-jung/Desktop/Junior/SCULPT/TestImages/black_c.jpg')

#displaying the target image for checking function
#im.show()

#converting the image to greyscale and resizing
grey = im.convert('L')
# size = 248, 128
size = 10, 10
grey = grey.resize(size,Image.LANCZOS)

#displaying the final to ensure it worked well
#grey.show()

#format is [width][height]
pixels = np.asarray(grey, dtype=np.uint8)

#cycling through and writing all pixel values to the PIC
count = 0
print("[ %s ] dimension of array" % str(pixels.shape))

pixIdx = np.indices((size[0], size[1]))
pixX = pixIdx[0].ravel()
pixY = pixIdx[1].ravel()
pixFlat = pixels.ravel()
largestVal = np.sort(pixFlat)[-1]
pixAll = np.column_stack((pixX, pixY, np.subtract(np.repeat(largestVal,
    pixFlat.size), pixFlat)))
print("imagedata")
print(pixAll)


#Reprinting to make copy/paste friendly
happy = str(list(pixAll)).replace("array(","").replace(")", "").replace('[',
        '\n{').replace(']', '}')
print(happy)

exit = "e\r"
#ser.write(str.encode(exit))
time.sleep(1)
#ser.close()
print("[ Done ] All pixels Loaded")
