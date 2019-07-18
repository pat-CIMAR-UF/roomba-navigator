#!/usr/bin/env python

import rospy
from time import sleep
from math import exp
from sensor_msgs.msg import Joy

import struct
import sys, glob # for listing serial ports

try:
    import serial
except ImportError:
    raise



port = '/dev/ttyUSB0'

forward = 0
reverse = 0
turn = 0
forward_reverse = 0

# ----------------------------------------------------------------
def joyCallback(data):
	#print('Axes_1: {:2.4}, Axes_2: {:2.4}, Axes_3: {:2.4}, Axes_4: {:2.4}, Axes_5: {:2.4}, Axes_6: {:2.4}'.format(data.axes[0],data.axes[1],data.axes[2],data.axes[3],data.axes[4],data.axes[5]))

	mapJoyToEfforts(data.axes)


def mapJoyToEfforts(axes):
	""" 
	Takes the joystick inputs and maps them to corresponding: throttle and braking efforts (0-100%), steering angle in 
	encoder counts
	"""
	global forward, reverse, turn, forward_reverse

	# Linear map [1, -1] to [0, 50] in percentage
	forward_reverse = axes[3]
	forward = 1/(1 + exp(5*(axes[5] - 1) + 5))*100
	reverse = 1/(1 + exp(5*(axes[2] - 1) + 5))*100
	turn = axes[0]

	return


# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
def sendCommandASCII(command):
	cmd = ""
	for v in command.split():
	    cmd += chr(int(v))

	sendCommandRaw(cmd)

# sendCommandRaw takes a string interpreted as a byte array
def sendCommandRaw(command):
	global connection

	try:
	    if connection is not None:
	        connection.write(command)
	    else:
	        print "Not connected."
	except serial.SerialException:
	    print "Lost connection"
	    connection = None

	#print ' '.join([ str(ord(c)) for c in command ])
	#self.text.insert(END, ' '.join([ str(ord(c)) for c in command ]))
	#self.text.insert(END, '\n')
	#self.text.see(END)

# getDecodedBytes returns a n-byte value decoded using a format string.
# Whether it blocks is based on how the connection was set up.
def getDecodedBytes(self, n, fmt):
	global connection
	
	try:
	    return struct.unpack(fmt, connection.read(n))[0]
	except serial.SerialException:
	    print "Lost connection"
	    connection = None
	    return None
	except struct.error:
	    print "Got unexpected data from serial port."
	    return None

# get8Unsigned returns an 8-bit unsigned value.
def get8Unsigned(self):
	return getDecodedBytes(1, "B")

# get8Signed returns an 8-bit signed value.
def get8Signed(self):
	return getDecodedBytes(1, "b")

# get16Unsigned returns a 16-bit unsigned value.
def get16Unsigned(self):
	return getDecodedBytes(2, ">H")

# get16Signed returns a 16-bit signed value.
def get16Signed(self):
	return getDecodedBytes(2, ">h")

def roombaTeleopNode():

	global forward, reverse, turn, connection, forward_reverse

	rospy.Subscriber("joy", Joy, joyCallback)

	# Initialize ROS Node	
	rospy.init_node('roomba_teleop', anonymous=True)

	print "Trying " + str(port) + "... "
	try:
		connection = serial.Serial(port, baudrate=115200, timeout=1)
		print "Connected!"
	except:
		print "Failed."

	sendCommandASCII('128') # P command
	sleep(0.1)
	sendCommandASCII('131') # S command

	lastTime = 0
	while not (rospy.is_shutdown()):
		currentTime = rospy.Time.now().to_nsec()
		# Calculate a time and then send messages at 20ms intervals	
		if((currentTime/1000.0 - lastTime/1000.0) > 20000):
			lastTime = currentTime
			#if(abs(forward) > abs(reverse)):
			#	velocity = 2*forward
			#else:
			#	velocity = -2*reverse

			velocity = 200*forward_reverse
			rotation = turn*300

			#print(forward, reverse, turn)

			# compute left and right wheel velocities
			vr = velocity + (rotation/2)
			vl = velocity - (rotation/2)

			# create drive command
			cmd = struct.pack(">Bhh", 145, vr, vl)
			sendCommandRaw(cmd)


if __name__ == '__main__':
	try:
		roombaTeleopNode()
	except rospy.ROSInterruptException:
		print("Program Failed")
		pass

