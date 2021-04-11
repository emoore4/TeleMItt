"""
TeleMitt - Application Code
===========================================
Author: Ethan Moore, Sam Irvine, Kyle Hoar, Aali Q Alotaibi
Date: April 11th, 2021
Description: This python code generates the GUI for the TeleMitt glove, as well as handles the software functionality of the rest of the application (Connecting to glove, 
Starting/Stopping the 3D model, writing the data to a csv file).
"""

from PIL import ImageTk,Image 
import tkinter as tk
from tkinter import *
from tkinter import filedialog as fd 
from tkinter import messagebox as mb
from tkinter.ttk import *
from serial import Serial
import threading
import sys, os
import csv
import time
from time import strftime
import datetime
import os.path
from os import path
import math


# Create the initial window used for the GUI.
window = tk.Tk() 
window.title("Serial Glove Capture")
window.resizable(width=False, height=False) # Do not make the GUI frame resizable. 

# Define Columns and rows to place the smaller windows for the GUI in. 
# These smaller frames are the serial connection window, the data capture control window, 
# the 3D model window as well as the calibration window.
window.columnconfigure([0, 1], minsize=300) 
window.rowconfigure([0, 1], minsize=200)
window.grid_columnconfigure(0, weight=1)
window.grid_columnconfigure(1, weight=1)

# Global Variable Initial Declarations
data = ''                                                   # List used to store the data being read in via the serial port.
buffer_create = ''                                          # List to store the data the 3D model references.
serial_object = None                                        # The pyserial object reperesenting the connection to the TeleMitt.
threads = []                                                # A list object used to store the different threads being initialized, so they can later be referenced and terminated.
model = ''                                                  # An object to represent the pathway for the .exe file used to initialize the model. 
var_seconds_counter = 0                                     # The value representing the amount of time elapsed since starting to record.
var_counter_control = False                                 # Control variable used to start/stop the timer on the gui.
var_serial_options = tk.StringVar(window)                   # The entry field used to select the connected com port in the gui.


# A function that opens a new window from an existing GUI and embeds a photo in the center of the new window.
# This function is used exclusively in the calibration buttons, to generate a new window with an image in the center representing
# the appropriate calibration hand position. 
# input: 
# windowName - String object representing the name of the newly created window.
# filepath - the file path for the image being placed in the new window.
# senddata - the value being written to the connected serial port to initiate the calibration of the glove.
def openNewWindow(windowName, filepath,senddata):
      
	newWindow = tk.Toplevel(window)                         # Toplevel object which will be treated as a new window
	newWindow.title("" + windowName)                        # sets the title of the Toplevel widget
	newWindow.geometry("460x400")                           # sets the geometry of toplevel
    
	temp_window = tk.Frame(newWindow)                       # Initialize a frame within the newly created window
	temp_window.pack(side=tk.TOP, fill=tk.BOTH, expand=1)

	temp_window.grid_columnconfigure([0], weight=1)         # Define a single column 
	temp_window.grid_rowconfigure([0,1,2,3],weight=1)       # Define 4 rows for headers, buttons and the image file.

    # Open and embed the image file found in the inputted file path to the newly created window.
    image1 = Image.open(filepath)                           
    test = ImageTk.PhotoImage(file = filepath)
    label1 = tk.Label(temp_window,image=test)
    label1.grid(row=1,sticky=tk.NSEW, padx=2, pady=2)
    label1.image = test

    # Define a tkinter label to show the calibration and GUI interaction steps.                                                             
	instruc = tk.Label(temp_window, text="Hold hand in position shown below until notified the calibration is complete.")
	instruc.grid(row=0,sticky=tk.NSEW, padx=2, pady=2)

    # A tkinter label showing the progress of the calibration.
	header = tk.Label(temp_window,text="Calibration under way...")
	header.grid(row=2,sticky=tk.NSEW, padx=2, pady=2)

    # The button to communicate to the glove that the hand is in the appropriate calibration position. 
	calib_button = tk.Button(temp_window,text="Enter",command = send(senddata))
	calib_button.grid(row=3,sticky=tk.NSEW, padx=2, pady=2)
	
	time.sleep(3)                                           # A delay to allow the serial communication time to transpire. 
	
    # If the returned data is a 3 (the value passed during the final calibration step)
    # change the instruction label to "begin calibration"
    if(senddata == "3"):
		header = tk.Label(text="Calibration complete. Close Window and begin your session.")
	else:
		header = tk.Label(text="Calibration step complete. Close Window to proceed.")

# A function used to apply twos compliment to a hexadecimal digit.
# input:
# hexstr - the hexadecimal string for the number
# bits - the number of bits for the final number
# output:
# value - the newly computed number.
def twos_complement(hexstr,bits):
    value = int(hexstr,16)
    if value & (1 << (bits-1)):
        value -= 1 << bits
    return value

# A function called to create two new csv files, one to store the data from the glove
# during recording, the other to act as a buffer for the 3D model. 
# The data storing csv file has column headers defined, and names the csv file based on the current time and 
# date. 
# output:
# file_create - the name for the csv file used to store data. 
def file_creator():

    # the column names for each of the sensor values being stored.
    column_names = ["Index Flex", "Middle Flex", "Ring Flex", "Pinky Flex", 
    "Thumb Flex", "Accel X", "Accel Y", "Accel Z", "Gyro X", "Gyro Y", "Gyro Z"]
    
    # Create the csv file and define an object to capture the current time.
    now =  datetime.datetime.now() # current date and time
    date_time = now.strftime("%m-%d-%Y--%H-%M-%S") # Data and time string format
    
    # Make the file names global.
    global file_create
    global buffer_create
    
    # Define the two file names.
    file_create = "TeleMittData--" + date_time + ".csv" 
    buffer_create = "ModelBuffer.csv"


    # Open and write column headers to CSV file.
    with open(file_create,"a+", newline='') as f:
        writer = csv.writer(f) # Define writer object
        writer.writerow(column_names)
    return file_create 

# A function that accepts the data read via the serial port from the glove, as well as the two csv file names
# the data is being written to.
# The buffer file has its data overwritten, so that the model doesn't need to sort through all of the rows from the data recording file
# to find the newest set of data. 
# The data storing file has its rows appended, so each new set of data is added as a row in the file.
# input:
# data - a 17 element list object representing the sensor values being passed as individual bytes. 
# x - the file name for the csv file the data is being stored in. 
# buffer - the csv file name for the buffer file used by the 3D model. 
def file_write(data,x,buffer):

    # Convert the data from individual bytes to 16 bit signed integer values. 
    # This decreases the elements in the array from 17 to 11. 
    sensor_data = [data[0],data[1],data[2],data[3],data[4], twos_complement((hex(data[5]) + hex(data[6]).lstrip("0x")),16),
    twos_complement(hex(data[7]) + hex(data[8]).lstrip("0x"), 16),twos_complement(hex(data[9]) + hex(data[10]).lstrip("0x"), 16),
    twos_complement(hex(data[11]) + hex(data[12]).lstrip("0x"), 16),twos_complement(hex(data[13]) + hex(data[14]).lstrip("0x"), 16),
    twos_complement(hex(data[15]) + hex(data[16]).lstrip("0x"), 16)]

    # Open and write new data to the data storing CSV file.
    with open(x,"a+", newline='') as f:
        writer = csv.writer(f) # Define writer object
        writer.writerow(sensor_data)
    f.close() # Close the csv file until it needs to be written into again 
    
    # Apply the quaternion computations to the formatted sensor data list, then 
    # write the converted values into the buffer csv file.
    bufferData = quatfilter(sensor_data)
    with open(buffer,"w", newline='') as d:
        d.truncate()
        writer = csv.writer(d) # Define writer object
        writer.writerow(bufferData)
    d.close()

# The function that controls the timer being displayed on the GUI while recording data. 
def update_timer():
    # Global variable declarations for the seconds counter object as well as the counter control variable.
    global var_seconds_counter
    global var_counter_control
    
    # Add a second onto the counter each loop.
    var_seconds_counter = var_seconds_counter + 1

    # Display the counter on the GUI.
    lbl_timer_counter["text"] = "Timer: {}".format(datetime.timedelta(seconds=var_seconds_counter))
    
    # If the counter control value is True, update the timer. 
    # Otherwise, the timer is set to 0:00:00.
    if (var_counter_control == True):
        window.after(1000, update_timer)
    else:
        lbl_timer_counter["text"] = "Timer: 0:00:00"

# The function that starts the thread used to capture the data, so the serial read function doesn't block the rest of the application. 
# This function also intializes the counting values.
def startCapture():
	global var_seconds_counter
	global var_counter_control
	var_counter_control = True
	var_seconds_counter = -1

    # Start the timer
	update_timer()

    # Initialize the thread used to read in the data, and assign it the read_data function.
	p1 = threading.Thread(target = read_data)
	p1.start()

    # Place the newly created thread into the threads appendix, to allow thread object to be referenced
    # when it is time to close the thread.
	threads.append(p1)

# This function sets the model pathway based on the input from the user in the GUI.
def model_set():
    global model
    model = fd.askopenfilename() 

# The function that runs the executable file for the 3D model based on the input from the 
# model_set function.
def model_Start():
    os.startfile(r"" + model)

# The function that terminates the hand model exe file. 
def model_stop():
    os.system(r"taskkill /f /im HandModel.exe")


# Get available serial communication ports for a windows PC.
def listComPorts():
    try:
        from serial.tools.list_ports import comports
    except ImportError:
        return None
    if comports:
        com_ports_list = list(comports())
        ports_list = []
        for port in com_ports_list:
            ports_list.append(port.name)
            ports_list.append(port.description)
        return ports_list

# The function used to connect to the serial port chosen from the drop down menu in the GUI. 
# This function creates the serial_object variable, representing the serial connection. 
def connect_serial_port():
	global serial_object
	serial_object = Serial(port=var_serial_options.get(), baudrate=115200, timeout=3)  
	mb.showinfo('Connected', 'Connected to ' + var_serial_options.get())

# The function used to close the previously established serial connection. 
def disconnect_serial_port():
    global serial_object
    global var_serial_options
    portnum = var_serial_options.get()
    serial_object.close()
    mb.showinfo('Disconnected', 'Disconnected from ' + portnum)

# The function that converts the IMU data to quaternions. 
# input: 
# data - the data list representing the values read in from the TeleMitt.
# output:
# quatdata - the data list with the IMU values (specifically the gyroscope values) converted into quaternions.
def quatfilter(data):

	
    # System constants
    deltat = 0.001                                      # sampling period in seconds (shown as 1 ms)
    gyroMeasError = math.pi * (2.0 / 180.0)             # gyroscope measurement error in rad/s (shown as 5 deg/s)
    beta = math.sqrt(3.0 / 4.0) * gyroMeasError         # compute beta

	#Global System Variables
    a_x = data[5]/1000
    a_y = data[6]/1000
    a_z = data[7]/1000                                  # accelerometer measurements
    w_x = data[8]*math.pi/180
    w_y = data[9]*math.pi/180
    w_z = data[10]*math.pi/180                          # gyroscope measurements in rad/s

    SEq_1 = 1.0
    SEq_2 = 0.0
    SEq_3 = 0.0
    SEq_4 = 0.0	                                        # estimated orientation quaternion elements with initial conditions

    halfSEq_1 = 0.5 * SEq_1
    halfSEq_2 = 0.5 * SEq_2
    halfSEq_3 = 0.5 * SEq_3
    halfSEq_4 = 0.5 * SEq_4
    twoSEq_1 = 2.0 * SEq_1
    twoSEq_2 = 2.0 * SEq_2
    twoSEq_3 = 2.0 * SEq_3
  
    # Normalise the accelerometer measurement
    norm = math.sqrt(a_x * a_x + a_y * a_y + a_z * a_z)
    a_x /= norm
    a_y /= norm
    a_z /= norm
  
    # Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y
    f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z 
    J_11or24 = twoSEq_3
    J_12or23 = 2.0 * SEq_4
    J_13or22 = twoSEq_1
    J_14or21 = twoSEq_2
    J_32 = 2.0 * J_14or21
    J_33 = 2.0 * J_11or24
 
    # Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2

    # Normalise the gradient
    norm = math.sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4)
    SEqHatDot_1 /= norm
    SEqHatDot_2 /= norm
    SEqHatDot_3 /= norm
    SEqHatDot_4 /= norm
  
    # Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x
  
    # Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat
 
    # Normalise quaternion
    norm = math.sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4)
    SEq_1 /= norm
    SEq_2 /= norm
    SEq_3 /= norm
    SEq_4 /= norm

    g = [0,0,0]
    g[0] = 2 * (SEq_2 * SEq_4 - SEq_1 * SEq_2)
    g[1] = 2 * (SEq_1 * SEq_2 + SEq_2 * SEq_4)
    g[2] = SEq_1 * SEq_1 - SEq_2 * SEq_2 - SEq_3 * SEq_3 + SEq_4 * SEq_4

    a_x -= g[0]
    a_y -= g[1]
    a_z -= g[2]

    quatdata = [data[0], data[1], data[2], data[3], data[4], SEq_1, SEq_2, SEq_3, SEq_4, a_x, a_y, a_z]

    return quatdata

# The function that reads in the data from the serial connection as a line of text, and then passes it to the
# file writing function. 
def read_data():
	x = file_creator()                                  # Create the csv files. 
	serial_object.readline()                            # Read the data in as a line of text. (the data being passed is terminated with a "\n")
	while True:
		if (serial_object.inWaiting()>0):               # If there is data waiting to be read from the serial port, read.
			data = serial_object.readline()
			file_write(data,x,buffer_create)        
		else:                                           # If there is no data ready to be read, loop again. 
			return

# The function used to write to the serial port. 
# input:
# data_send - the values being written to the serial port. 
def send(data_send):
	y = bytes(data_send, 'utf-8')                       # Convert the data into bytes.
	serial_object.write(y)                              # Write the converted data to the connected serial port. 
	time.sleep(0.05)                                    # Delay to allow the data time to be written. 

# The function that is tied to the button calibrate step 1.
# This function passes values to the openNewWindow function, specific to the first calibration step.
def calibration_phase1():
    name = "Calibration Step 1"
    path = r"C:\Users\Kyle\Downloads\calibration1.jpg"
    openNewWindow(name,path,"1")

# The function that is tied to the button calibrate step 2.
# This function passes values to the openNewWindow function, specific to the second calibration step.
def calibration_phase2():
    name = "Calibration Step 2"
    path = r"C:\Users\Kyle\Downloads\calibration12.jpg"
    openNewWindow(name,path,"2")

# The function that is tied to the button calibrate step 3.
# This function passes values to the openNewWindow function, specific to the third calibration step.
def calibration_phase3():
    name = "Calibration Step 3"
    path = r"C:\Users\Kyle\Downloads\calibration2.gif"
    openNewWindow(name,path,"3")

# The function that terminates the string used to read in the serial data and terminates the timer. 
def stopCapture():
    global var_counter_control                      
    var_counter_control = False                         # Stop the timer

    threads[0].join()                                   # Terminate the thread created in startCapture


# Setup Frames
frm_top_left = tk.LabelFrame(window, text="Serial Configuration")
frm_top_right = tk.LabelFrame(window, text="Capture Window")
frm_bot_left = tk.LabelFrame(window, text="Model Control")
frm_bot_right = tk.LabelFrame(window, text="Calibration")
frm_top_left.grid(row=0, column=0, sticky=tk.NSEW, padx=2, pady=2)
frm_top_right.grid(row=0, column=1, sticky=tk.NSEW, padx=2, pady=2)
frm_bot_left.grid(row=1, column=0, sticky=tk.NSEW, padx=2, pady=2)
frm_bot_right.grid(row=1, column=1, sticky=tk.NSEW, padx=2, pady=2)

# Top Left Frame - Serial Control Menu
frm_serial_menu = tk.Frame(frm_top_left)
frm_serial_menu.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
frm_serial_menu.grid_columnconfigure([0, 1, 2], weight=1)
frm_serial_menu.grid_rowconfigure([0, 1, 2], weight=1)
lbl_serial_list = tk.Label(frm_serial_menu,  text="Serial Port: ")
lbl_serial_list.grid(row=0, column=0, sticky=tk.NSEW, padx=2, pady=2)
OptionList = listComPorts()
var_serial_options.set(OptionList[0])
opt_serial = tk.OptionMenu(frm_serial_menu, var_serial_options, *OptionList)
opt_serial.grid(row=0, column=1, sticky=tk.NSEW, padx=2, pady=2)
btn_connect = tk.Button(frm_serial_menu,  text="Connect",command=connect_serial_port)
btn_connect.grid(row=0, column=2, sticky=tk.NSEW, padx=2, pady=2)
btn_disconnect = tk.Button(frm_serial_menu, text = "Disconnect", command = disconnect_serial_port)
btn_disconnect.grid(row = 1, column = 2, sticky=tk.NSEW, padx=2, pady=2)
lbl_timer_counter = tk.Label(frm_serial_menu,  text="Timer: 0:00:00")
lbl_timer_counter.grid(row=2, column=0, sticky=tk.NSEW, padx=2, pady=2)


# Bottom Left Frame - Model Control
mode_control_frame = tk.Frame(frm_bot_left)
mode_control_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
mode_control_frame.grid_columnconfigure([0], weight=1)
mode_control_frame.grid_rowconfigure([0,1,2,3],weight=1)
lbl_imu_acc_x = tk.Label(mode_control_frame,  text="Enter file path for HandModel.exe")
lbl_imu_acc_x.grid(row=0, column=0, sticky=tk.NSEW, padx=2, pady=2)
btn_modelEntry = tk.Button(mode_control_frame,text="Choose the 3D model Executable",command=model_set)
btn_modelEntry.grid(row=1, column=0,sticky=tk.NSEW, padx=2, pady=2)
btn_modelStart = tk.Button(mode_control_frame,text="Start 3D Model",command=model_Start)
btn_modelStart.grid(row=2, column=0,sticky=tk.NSEW, padx=2, pady=2)
btn_modelStop = tk.Button(mode_control_frame,text="Stop 3D Model",command=model_stop)
btn_modelStop.grid(row=3, column=0,sticky=tk.NSEW, padx=2, pady=2)

# Top Right Frame - Capture Control
capture_control_frame = tk.Frame(frm_top_right)
capture_control_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
capture_control_frame.grid_columnconfigure([0], weight=1)
capture_control_frame.grid_rowconfigure([0,1,3,4], weight = 1)
btn_start = tk.Button(capture_control_frame,text="Start Capture", command = startCapture)
btn_start.grid(row=1,sticky=tk.NSEW, padx=2, pady=2)
btn_stop = tk.Button(capture_control_frame,text="Stop Capture", command = stopCapture)
btn_stop.grid(row=3,sticky=tk.NSEW, padx=2, pady=2)
capture_space_holder1= tk.Label(capture_control_frame)
capture_space_holder1.grid(row=4, sticky=tk.NSEW, padx=2, pady=2)


# Bottom Right Frame - Calibration Control
calibration_frame = tk.Frame(frm_bot_right)
calibration_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=1)
calibration_frame.grid_columnconfigure([0], weight=1)
calibration_frame.grid_rowconfigure([0, 1, 2],weight=1)
btn_calibrate1 = tk.Button(calibration_frame,text="Calibrate Glove Step 1", command = calibration_phase1)
btn_calibrate1.grid(row = 0,sticky=tk.NSEW, padx=2, pady=2)
btn_calibrate2 = tk.Button(calibration_frame,text="Calibrate Glove Step 2", command = calibration_phase2)
btn_calibrate2.grid(row = 1,sticky=tk.NSEW, padx=2, pady=2)
btn_calibrate3 = tk.Button(calibration_frame,text="Calibrate Glove Step 3", command = calibration_phase3)
btn_calibrate3.grid(row = 2,sticky=tk.NSEW, padx=2, pady=2)


# Run the application
if __name__ == "__main__":    
    window.mainloop()

