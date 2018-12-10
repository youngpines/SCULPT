#!/usr/bin/env python3
import serial
import time

############################ [ Constants ] ####################################
REST_TIME = 5 # Time in seconds between setting values
NUM_ROWS  = 5 # Number of rows to draw for 
NUM_COLS  = 5 # Number of cols to draw for

############################ [ Functions ] ####################################
# Write serial from a list's row and column
def write_serial( freq_list, row, col ):
    # Open the Serial port
    ser = serial.Serial("/dev/ttyUSB0") 
    print( "Serial [ %s ] is opened" % ser.name ) # check the serial port
    frequency = freq_list[row][col]
    message = "f {}\r".format( frequency )
    print( "[ (%d, %d) ] Going to write [ %d ]" % (row, col, frequency) )
    status = ser.write(message.encode('utf-8'))
    ser.close()
    print( "Serial [ %s ] is closed" % ser.name )

# Write serial for a specific number
def write_serial_manual( frequency ):
    # Open the Serial port
    ser = serial.Serial("/dev/ttyUSB0") 
    print( "Serial [ %s ] is opened" % ser.name ) # check the serial port
    message = "p {}\r".format( frequency )
    print( "Going to write [ %d ]" % frequency )
    status = ser.write(message.encode('utf-8'))
    ser.close()
    print( "Serial [ %s ] is closed" % ser.name )

# Generate an "empty" list with NUM_ROWS x NUM_COLS of 0's
def generate_empty_list():
    empty_list = [ [ 0 for row in range(NUM_ROWS) ] for col in range(NUM_COLS) ]
    return empty_list

# Populate the frequency list from digits from 'filename'
# TODO - Make this more robust with os library
def populate_freq_list( filename ):
    freq_list = generate_empty_list()
    with open( filename ) as f:
        row = 0
        for line in f:
            if (row < NUM_ROWS):
                freq_list[row] = list( map(int, line.split()) )[ : NUM_COLS ]
                row += 1
    return freq_list
    
# Flattens a 2D list
def flatten_freq_list():
    flattened = [ val for row in freq_list for val in row ]
    print( "The flattened list is [ %s ]" % ", ".join(map(str, flattened)) )
    return flattened

############################## [ Main ] #######################################
def main():
    freq_list = populate_freq_list( "map.txt" )
    #flattened = flatten_freq_list() 
    #for freq in freq_list:
    #    write_serial( row, col )
    #    time.sleep( REST_TIME ) 
    for row in range( NUM_ROWS ):
        for col in range( NUM_COLS ):
            write_serial( freq_list, row, col )
            time.sleep( REST_TIME )

############################ [ Main Call ] ####################################
if ( __name__ == "__main__" ):
    main()
