#!/usr/bin/python

'''
Project:
	Smart Mobile Air Purifier (SMAP) Prototype

Title:
	Mobile Platform Parameters and Registers

Description
	-set parameters for serial communication
	-set parameters for controlling motors
	-set parameters for computing the robot kinematics
	-set motors' mechanical properties

* Copyrighted by Mingyo Seo
* Created on October 13 2018
'''



## Serial Communication Parameter ##

# Communication Configuration
SER_BAUDRATE = 115200
# SER_DEVICE = "/dev/tty.SLAB_USBtoUART"
SER_DEV_DRV = "/dev/ttyUSB1"
SER_DEV_FAN = "/dev/ttyUSB0"

SER_DRV_FAN = "/dev/ttyUSB0"
SER_SENSOR  = "/dev/ttyAMA0"

SER_TIMEOUT = 0.1

TYPE_DEV_DRV = 1
TYPE_DEV_FAN = 2
TYPE_DEV_SEN = 3


# Communication Protocol
MSG_START = 'a'
MSG_ENABLE = 'e' 
MSG_CONTROL = 'c' 
MSG_REQUEST = 'r' 
MSG_END = 'q'

MSG_DRV_ENABLE = 'e'
MSG_DRV_CONTROL = 'c'
MSG_DRV_REQUEST = 'r'
MSG_FAN_ENABLE = 'f'
MSG_FAN_CONTROL = 'g'
MSG_FAN_REQUEST = 'h'

MSG_RESOLUTION = 4
MSG_DEV_NUMBER = 2
MSG_LENGTH = 3 + MSG_DEV_NUMBER * MSG_RESOLUTION

## Mechanical Parameter ##
GMT_HALF_WIDTH = 205.0
GMT_HALF_LENGTH = 290.0
GMT_WHEEL_RADIUS = 70.0

## Motor Command Parameter ##
DRV_OFFSET = 0
DRV_MAX = 150
DRV_MIN = -150

## Fan Command Parameter ##
FAN_OFFSET = 0
FAN_MAX = 150
FAN_MIN = 30