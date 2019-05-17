#!/usr/bin/env python3
###################### [ IMPORTS ] ############################################ 
from PIL import Image
import numpy as np
import serial, time
import argparse

###################### [ FUNCTIONS ] ########################################## 
def openSerial(port, baud, debug):
    """ Opens the Serial port
    Parameters:
    port  (str): port path (e.g. "/dev/tty/USB0")
    baud  (int): baud rate to open Serial communication with
    debug (int): turn debug mode on (1) or off (0)

    Returns:
    None
    """
    # Open the Serial Port
    ser = serial.Serial(port, baudrate = baud,   \
                        parity = serial.PARITY_NONE, \
                        stopbits = 1, bytesize = 8)
    if (debug):
        print("Using [ " + ser.name + " ]")
    return ser

def matprint(mat, fmt="g"):
    """ Prints a nice, visual version of the carved image with pixel values
    Parameters:
    mat (numpy array): array to print out nicely

    Returns:
    None
    """
    col_maxes = [max([len(("{:"+fmt+"}").format(x)) for x in col]) for col in mat.T]
    for x in mat:
        for i, y in enumerate(x):
            print(("{:"+str(col_maxes[i])+fmt+"}").format(y), end="  ")
        print("")


def processImage(imageLoc, xDim, yDim, debug):
    """ Processes image to greyscale pixel array
    Parameters:
    imageLoc (str): absolute path to image to process
    xDim     (int): x dimension of pixel grid
    yDim     (int): y dimension of pixel grid
    debug    (int): turn debug mode on (1) or off (0)

    Returns:
    pixGrid    (numpy array): (xDim*yDim) x 3 array of pixel 
                              greyscale values (xCoord, yCoord, greyscale)
    largestVal (int): largest greyscale value of converted pixel image array
    """
    # Open up the target image
    im = Image.open(imageLoc)
    
    #Displaying the target image for checking function
    im.show()
    
    # Converting the image to greyscale and resizing
    grey = im.convert('L')
    size = yDim, xDim
    grey = grey.resize(size,Image.LANCZOS)
    
    # Display the greyscale image
    grey.show()
    
    # Convert greyscale pixels as an array
    pixels = np.asarray(grey, dtype=np.uint8)
    # Assign (x, y) coordinates to each pixel
    pixGridIdx = np.indices((pixels.shape[0], pixels.shape[1]))
    pixX = pixGridIdx[0].ravel()
    pixY = pixGridIdx[1].ravel()
    # "Unfold" pixel array for sending to serial as 1 stream
    pixVal = pixels.ravel()
    # "Invert" the image so that 0 = highest, 255 = deepest lvl on bed
    largestVal = np.sort(pixVal)[-1]
    inverted = np.subtract(np.repeat(largestVal, pixVal.size), pixVal)
    # Create pixel grid and grab largest value to return
    pixGrid = np.column_stack((pixX, pixY, inverted))
    largestVal = np.sort(inverted)[-1]
    # Cycle through and writing all pixel values to the PIC
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
    """ Sends pixels over serial connection
    Parameters:
    ser        (Serial object): Serial object to write data too
    pixelArray (numpy array): greyscale image pixel array
    xDim       (int): x dimension of pixel grid
    yDim       (int): y dimension of pixel grid
    largestVal (int): largest greyscale value of converted pixel image array
    debug      (int): turn debug mode on (1) or off (0)

    Returns:
    None
    """
    # Write stats 's' (x dim, y dim, largest val) to Serial & opt. debug also 
    pixelStats = "s %d %d %d\r" % (xDim, yDim, largestVal)
    if (debug):
        print("Writing Stats [ %s ]" % pixelStats[:-1])
    ser.write(str.encode(pixelStats))
    time.sleep(0.5) # Wait a little
    # Send pixel array data over Serial starting with char 'p'
    for i in range(pixelArray.shape[0]):
        pixelData = "p %d %d %d\r" % (pixelArray[i][0], pixelArray[i][1],
                pixelArray[i][2])
        if (debug):
            print("Writing [ %s ]" % pixelData[:-1])
        ser.write(str.encode(pixelData))
        time.sleep(0.02)
    # Finished sending data, signal end with 'e'
    exit = "e\r"
    time.sleep(1)
    ser.write(str.encode(exit))
    time.sleep(0.5) 
    # Close the connection and notify user
    ser.close()
    print("[ Done ] All pixels Loaded")

###################### [ MAIN ] ############################################### 
def main():
    # Parse arguments provuded by user
    parser = argparse.ArgumentParser(description="SCULPT Image Processor")
    parser.add_argument("-p", action="store", dest="port", required=True, \
            help="Port UART connection is opened (e.g. /dev/ttyUSB0")
    parser.add_argument("-i", action="store", dest="file", required=True, \
            help="Absolute path  for image file to upload")
    parser.add_argument("-b", action="store", dest="baud", \
            help="Baudrate the  send pixels at", default=9600)
    parser.add_argument("-x", action="store", dest="xdim", \
            type=int, default=39, help="X dimension size")
    parser.add_argument("-y", action="store", dest="ydim", \
            type=int, default=47, help="Y dimension size")
    parser.add_argument("-d", action="store_false", default=True, dest="debug",
            help="Turn Debug Mode Off")
    r = parser.parse_args()
    
    ser = openSerial(r.port, r.baud, r.debug)
    pixelArray, largestVal = processImage(r.file, r.xdim, r.ydim, r.debug)
    sendPixels(ser, pixelArray, r.xdim, r.ydim, largestVal, r.debug)

if __name__ == "__main__":
    print("******[ Sculpt Start ]******")
    main()
    print("******[ Sculpt End ]******")
