# Importing the basic Image library 
from PIL import Image
import numpy, serial, time

BAUDRATE = 9600

def openSerial(port):
  ser = serial.Serial(port, baudrate = BAUDRATE, parity = serial.PARITY_NONE,
          stopbits = 1, bytesize = 8)
  print("Using [ " + ser.name + " ]")

def processImage(imageLoc):
    # Opening up the target image
    im = Image.open(imageLoc)
    
    #displaying the target image for checking function
    im.show()
    
    #converting the image to greyscale and resizing
    grey = im.convert('L')
    # size = 248, 128
    size = 128, 128
    grey = grey.resize(size,Image.LANCZOS)
    
    #displaying the final to ensure it worked well
    grey.show()
    
    #format is [width][height]
    pixels = numpy.asarray(grey, dtype=numpy.uint8)
    
    #cycling through and writing all pixel values to the PIC
    print("[ %s ] dimension of array" % str(pixels.shape))
    return pixels

def sendPixels(pixelArray):
    for i in range(size[1]):
        for j in range(size[0]):
            print("[ %d ] out of [ %d ] uploading" % (count, (size[0]*size[1])))
            val = "p " + str(pixels[i][j]) + "\r"
            print("Writing [ %s ]" % val[ : -1])
            ser.write(str.encode(val))
            time.sleep(0.03)
    exit = "e\r"
    ser.write(str.encode(exit))
    time.sleep(1)
    ser.close()
    print("[ Done ] All pixels Loaded")

##########################
def main():
    openSerial("/dev/ttyUSB0")
    pixelArray = processImage("/home/ho-jung/Downloads/black_c.jpg")
    sendPixels(pixelArray)

if __name__ == "__main__":
    print("******[ Sculpt Start ]******")
    main()
