#!/usr/bin/env python3
import serial
import time

REST_TIME = 5
NUM_ROWS = 5
NUM_COLS = 5
freq_list = [ [ 0 for row in range(NUM_ROWS) ] for col in range(NUM_COLS) ]

def write_serial( frequency ):
    # Open the Serial port
    ser = serial.Serial("/dev/ttyUSB0") 
    print( "Serial [ %s ] is opened" % ser.name ) # check the serial port
    message = "f {}\r".format( frequency )
    print( "Going to write [ %d ]" % frequency )
    status = ser.write(message.encode('utf-8'))
    ser.close()
    print( "Serial [ %s ] is closed" % ser.name )

def create_flattened_list( filename ):
    with open( filename ) as f:
        row = 0
        for line in f:
            if (row < NUM_ROWS):
                freq_list[row] = list( map(int, line.split()) )[ : NUM_COLS ]
                row += 1
    
    flattened = [ val for row in freq_list for val in row ]
    return flattened

def main():
    flattened = create_flattened_list( "map.txt" ) 
    print( "The flattened list is [ %s ]" % ", ".join(map(str, flattened)) )
    for freq in flattened:
        write_serial( freq )
        time.sleep( REST_TIME )

if ( __name__ == "__main__" ):
    main()
