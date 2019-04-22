#!/usr/bin/env python3

# Importing the basic Image library 
from PIL import Image
import numpy, serial, time, argparse

def openSerial(port, baud, debug):
    # Open the Serial Port
    ser = serial.Serial(port, baudrate = baud,   \
                        parity = serial.PARITY_NONE, \
                        stopbits = 1, bytesize = 8)
    if (debug):
        print("Using [ " + ser.name + " ]")

def processImage(imageLoc, xDim, yDim, debug):
    # Opening up the target image
    im = Image.open(imageLoc)
    
    #displaying the target image for checking function
    im.show()
    
    #converting the image to greyscale and resizing
    grey = im.convert('L')
    size = xDim, yDim
    grey = grey.resize(size,Image.LANCZOS)
    
    #displaying the final to ensure it worked well
    grey.show()
    
    #format is [width][height]
    pixels = numpy.asarray(grey, dtype=numpy.uint8)
    
    #cycling through and writing all pixel values to the PIC
    if (debug):
        print("[ %s ] dimension of array" % str(pixels.shape))
    return pixels

def sendPixels(pixelArray, debug):
    for i in range(size[1]):
        for j in range(size[0]):
            val = "p " + str(pixels[i][j]) + "\r"
            if (debug):
                print("[ %d ] / [ %d ] uploading" % (count, (size[0]*size[1])))
                print("Writing [ %s ]" % val[ : -1])
            ser.write(str.encode(val))
            time.sleep(0.03)
    exit = "e\r"
    ser.write(str.encode(exit))
    time.sleep(1)
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
            type=int, default=128, help="X dimension size")
    parser.add_argument("-y", action="store", dest="ydim", \
            type=int, default=128, help="Y dimension size")
    parser.add_argument("-d", action="store_false", default=True, dest="debug",
            help="Turn Debug Mode Off")
    r = parser.parse_args()
    
#    testParser(r.port, r.file, r.baud, r.xdim, r.ydim, r.debug)
    openSerial(r.port, r.baud, r.debug)
    pixelArray = processImage(r.file, r.xdim, r.ydim, r.debug)
    sendPixels(pixelArray, r.debug)

if __name__ == "__main__":
    print("******[ Sculpt Start ]******")
    main()
    print("******[ Sculpt End ]******")
