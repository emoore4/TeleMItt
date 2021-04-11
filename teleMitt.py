"""
TeleMitt Python Module
===========================================
Author: TeleMitt Group - Ethan Moore, Aali Q Alotaibi, Sam Irvine, Kyle Hoar
Date: April 6th, 2021
Description: This Module allows the user to serially connect to the TeleMitt and then read the data being passed as a list. 
The product will still need to be calibrated prior to using this module. Follow the calibration steps outlined by the application program. 
"""

from serial import Serial
import time
serial_object = None

# Creates a serial object that connects to the com port used by the TeleMitt.
# Input: 
# ================
# portnum - Com port number (integer)
# baudRate - baudRate used in the serial connection. Can be specified in the included embedded code.
def connect_serial_port(portnum, baudRate):
    global serial_object # Define the serial object
    portConnect = str(portnum) # Cast the inputted integer value as a string
    serial_object = Serial(port="COM"+portConnect, baudrate=baudRate, timeout=3) # Initialize the serial object
    time.sleep(0.5) # Allow the port time to connect

# Creates a serial object that connects to the com port used by the TeleMitt.
# Input: portnum - Com port number (integer)
# Output: Returns a boolean, True if disconnect was successful, False if not. 
def disconnect_serial_port(portnum):
    global serial_object
    if serial_object.close():
        return True
    else:
        return False

# Used in converting the separated 2-Byte integer values back to a signed 16 bit integer.
# Input:
# ===============
# hexstr - A hex string object.
# bits - the number of bits for the resulting integer value.
# Output: Returns the converted integer value.
def twos_complement(hexstr,bits):
    value = int(hexstr,16)
    if value & (1 << (bits-1)):
        value -= 1 << bits
    return value

# Reads and converts the data from the TeleMitt via the created serial object. 
# Output: Returns a list with 11 Integer elements, each one representing a sensor value. 
# The first 5 values are the normalize voltage outputs for the flex sensor circuits along each digit. 
# The remaining 6 values are the 6 IMU values, the layout for each is outlined below. 
# sensor_data[0] = Accelerometer
# sensor_data[1] = Accelerometer
# sensor_data[2] = Accelerometer
# sensor_data[3] = Gyroscope
# sensor_data[4] = Gyroscope
# sensor_data[5] = Gyroscope
# sensor_data[6] = Index Finger Flex Sensor 
# sensor_data[7] = Middle Finger Flex Sensor
# sensor_data[8] = Ring Finger Flex Sensor 
# sensor_data[9] = Pinky Finger Flex Sensor 
# sensor_data[10] = Thumb Flex Sensor
def read_data():
    serial_object.readline()
    if (serial_object.inWaiting()>0):
        data = serial_object.readline()
        sensor_data = [data[0],data[1],data[2],data[3],data[4], twos_complement((hex(data[5]) + hex(data[6]).lstrip("0x")),16),
        twos_complement(hex(data[7]) + hex(data[8]).lstrip("0x"), 16),twos_complement(hex(data[9]) + hex(data[10]).lstrip("0x"), 16),
        twos_complement(hex(data[11]) + hex(data[12]).lstrip("0x"), 16),twos_complement(hex(data[13]) + hex(data[14]).lstrip("0x"), 16),
        twos_complement(hex(data[15]) + hex(data[16]).lstrip("0x"), 16)]
    return sensor_data

