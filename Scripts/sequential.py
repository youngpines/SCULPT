# Importing the basic Image library 
from PIL import Image
import numpy as np
import serial, time

import sys
## Opening up a serial port to transmit to the Microcontroller
#ser = serial.Serial(
#    port='/dev/ttyUSB0',        #the port can change based on which is used
#    baudrate=9600,
#    parity= serial.PARITY_NONE,
#    stopbits=1,
#    bytesize=8
#)
#print("Using [ " + ser.name + " ]")

# Opening up the target image
im = \
Image.open('/home/ho-jung/Desktop/Junior/SCULPT/TestImages/fish.png')
#displaying the target image for checking function
#im = im.rotate(90)
im.show()

#converting the image to greyscale and resizing
grey = im.convert('L')
#size = 45, 38 
size = 2, 3
grey = grey.resize(size,Image.LANCZOS)
np.set_printoptions(threshold=sys.maxsize)
#displaying the final to ensure it worked well
grey.show()

#format is [width][height]
pixels = np.asarray(grey, dtype=np.uint8)
print(pixels.shape)

#cycling through and writing all pixel values to the PIC
count = 0
print("[ %s ] dimension of array" % str(pixels.shape))

pixIdx = np.indices((pixels.shape[0], pixels.shape[1]))
pixX = pixIdx[0].ravel()
pixY = pixIdx[1].ravel()
pixFlat = pixels.ravel()
largestVal = np.sort(pixFlat)[-1]
inverted =  np.subtract(np.repeat(largestVal, pixFlat.size), pixFlat)
pixAll = np.column_stack((pixX, pixY, inverted))
largestVal = np.sort(inverted)[-1]
print("After, largest value")
print(largestVal)
print("imagedata")
print(pixAll)

pixVis = np.resize(pixAll[:, 2], (pixels.shape[0], pixels.shape[1]))
pixXVis = np.resize(pixAll[:, 0], (pixels.shape[0], pixels.shape[1]))

#Reprinting to make copy/paste friendly
happy = str(list(pixAll)).replace("array(","").replace(")", "").replace('[',
        '\n{').replace(']', '}')
print(happy)
print("\r")

def matprint(mat, fmt="g"):
    col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
    for x in mat:
        for i, y in enumerate(x):
            print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end="  ")
        print("")

matprint(pixVis)
print("\r")
pixelStats = "s %d %d %d\r" % (size[0], size[1], largestVal)
print("Writing Stats [ %s ]" % pixelStats[:-1])
#ser.write(str.encode(pixelStats))
for i in range(pixAll.shape[0]):
    pixelData = "p %d %d %d\r" % (pixAll[i][0], pixAll[i][1], pixAll[i][2])
    print("Writing [ %s ]" % pixelData[:-1])
#    ser.write(str.encode(pixelData))
#    time.sleep(0.18)
exit = "e\r"
#time.sleep(1)
#ser.write(str.encode(exit))
#time.sleep(0.5)
#ser.close()
print("[ Done ] All pixels Loaded")
