#!/usr/bin/env python3

# Importing the basic Image library 
from PIL import Image
import numpy as np
import serial, time
import argparse

def openSerial(port, baud, debug):
    # Open the Serial Port
    ser = serial.Serial(port, baudrate = baud,   \
                        parity = serial.PARITY_NONE, \
                        stopbits = 1, bytesize = 8)
    if (debug):
        print("Using [ " + ser.name + " ]")
    return ser

def matprint(mat, fmt="g"):
    col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
    for x in mat:
        for i, y in enumerate(x):
            print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end="  ")
        print("")


def processImage(imageLoc, xDim, yDim, debug):
    # Opening up the target image
    im = Image.open(imageLoc)
    
    #displaying the target image for checking function
    im.show()
    
    #converting the image to greyscale and resizing
    grey = im.convert('L')
    size = yDim, xDim
    grey = grey.resize(size,Image.LANCZOS)
    
    #displaying the final to ensure it worked well
    grey.show()
    
    pixels = np.asarray(grey, dtype=np.uint8)
    pixGridIdx = np.indices((pixels.shape[0], pixels.shape[1]))
    pixX = pixGridIdx[0].ravel()
    pixY = pixGridIdx[1].ravel()
    pixVal = pixels.ravel()
    largestVal = np.sort(pixVal)[-1]
    inverted = np.subtract(np.repeat(largestVal, pixVal.size), pixVal)
    pixGrid = np.column_stack((pixX, pixY, inverted))
    largestVal = np.sort(inverted)[-1]
    #cycling through and writing all pixel values to the PIC
    if (debug):
        print("[ %s ] dimension of array\r" % str(pixels.shape))
        print("[ %s ] largest value\r" % str(largestVal))
        print("image data")
        print(pixGrid)
        print("\r")
        pixVis = np.resize(pixGrid[:, 2], (pixels.shape[0], pixels.shape[1]))
        matprint(pixVis)
    return pixGrid, largestVal

def sendPixels(ser, pixelArray, xDim, yDim, largestVal, debug):
    pixelStats = "s %d %d %d\r" % (xDim, yDim, largestVal)
    if (debug):
        print("Writing Stats [ %s ]" % pixelStats[:-1])
    ser.write(str.encode(pixelStats))
    time.sleep(0.5)
    for i in range(pixelArray.shape[0]):
        pixelData = "p %d %d %d\r" % (pixelArray[i][0], pixelArray[i][1],
                pixelArray[i][2])
        if (debug):
            print("Writing [ %s ]" % pixelData[:-1])
        ser.write(str.encode(pixelData))
        #time.sleep(0.015)
        time.sleep(0.02)
    exit = "e\r"
    time.sleep(1)
    ser.write(str.encode(exit))
    time.sleep(0.5)
    ser.close()
    print("[ Done ] All pixels Loaded")


def testParser(port, image, baud, x, y, d):
    print("Port  = [ " + port      + ", " + str(type(port))  + " ]")
    print("Image = [ " + image     + ", " + str(type(image)) + " ]")
    print("Baud  = [ " + str(baud) + ", " + str(type(baud))  + " ]")
    print("X Dim = [ " + str(x)    + ", " + str(type(x))     + " ]")
    print("Y Dim = [ " + str(y)    + ", " + str(type(x))     + " ]")
    print("Debug = [ " + str(d)    + ", " + str(type(d))     + " ]")
    if (d):
        print("Debug mode!")
    else:
        print("No debug mode")

##########################
def main():
    parser = argparse.ArgumentParser(description="SCULPT Image Processor")
    parser.add_argument("-p", action="store", dest="port", required=True, \
            help="Port UART connection is opened (e.g. /dev/ttyUSB0")
    parser.add_argument("-i", action="store", dest="file", required=True, \
            help="Absolute path  for image file to upload")
    parser.add_argument("-b", action="store", dest="baud", \
            help="Baudrate the  send pixels at", default=9600)
    parser.add_argument("-x", action="store", dest="xdim", \
            type=int, default=45, help="X dimension size")
    parser.add_argument("-y", action="store", dest="ydim", \
            type=int, default=38, help="Y dimension size")
    parser.add_argument("-d", action="store_false", default=True, dest="debug",
            help="Turn Debug Mode Off")
    r = parser.parse_args()
    
#    testParser(r.port, r.file, r.baud, r.xdim, r.ydim, r.debug)
    ser = openSerial(r.port, r.baud, r.debug)
    pixelArray, largestVal = processImage(r.file, r.xdim, r.ydim, r.debug)
    sendPixels(ser, pixelArray, r.xdim, r.ydim, largestVal, r.debug)

if __name__ == "__main__":
    print("******[ Sculpt Start ]******")
    main()
    print("******[ Sculpt End ]******")
