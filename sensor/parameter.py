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

## Message
SONAR_T_FRAME_ID = "base_sonar_top"
SONAR_B_FRAME_ID = "base_sonar_bottom"
IMU_FRAME_ID = "link_imu"

## Sonar
## http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html
SONAR_T_FIELD_OF_VIEW = 0.875 # 1.0472 # 60 degrees (full of arc)
SONAR_B_FIELD_OF_VIEW = 1.047 # 1.22173 # 70 degrees (full of arc)
SONAR_T_MIN_RANGE = 0.1
SONAR_B_MIN_RANGE = 0.1
SONAR_T_MAX_RANGE = 0.8 # 0.30
SONAR_B_MAX_RANGE = 0.2 # 0.15

SONAR_HORN_FIELD_OF_VIEW = [0.6733496388, 0.564514844, 0.5398300729, 0.5134166609, 0.5087361171, 0.4899573263, 0.48457557, 0.4545105212, 0.5212047835]
SONAR_DEF_FIELD_OF_VIEW = [0.7610127542, 0.6284637982, 0.7610127542, 0.6733496388, 0.6554770136, 0.6524875685, 0.5959938472, 0.559899645, 0.6032380925]

#SONAR_T_FIELD_OF_VIEWS = SONAR_HORN_FIELD_OF_VIEW
SONAR_T_FIELD_OF_VIEWS = SONAR_DEF_FIELD_OF_VIEW
SONAR_B_FIELD_OF_VIEWS = SONAR_DEF_FIELD_OF_VIEW

## Serial Communication Parameter ##

# Communication Configuration
SER_BAUDRATE = 115200

## IF RASPBERRY_PI
SER_DEVICE = "/dev/ttyAMA0"
## ELSE
# SER_DEVICE = "/dev/ttyUSB1"
## END_IF

SER_TIMEOUT = 0.1

# Communication Protocol
MSG_START = 't'
MSG_ENABLE = 'e' 
MSG_SONAR = 's'
MSG_DUST = 'd'
MSG_IMU = 'i'
MSG_END = 'q'

MSG_WIFI_ENABLE = 'w'
MSG_WIFI_START = '<'
MSG_WIFI_END = '>'
MSG_WIFI_MAXIMUM = 500

MSG_RESOLUTION = 4
MSG_DEV_NUMBER = 1
MSG_LENGTH = 9  # 5 + MSG_DEV_NUMBER * MSG_RESOLUTION


# tb0|4132q


IMU_IGNORE_COUNT = 50


# Covarience

COV_AX = 3.676657530528100E-07
COV_AY = 2.853356311666220E-07
COV_AZ = 3.108230152761650E-07

COV_ANGULAR = [
	COV_AX, 0.0, 0.0,
	0.0, COV_AY, 0.0,
	0.0, 0.0, COV_AZ
]

COV_LX = 1.036906039894480E-04
COV_LY = 1.013369945998860E-04
COV_LZ = 2.235569510522100E-04

COV_LINEAR = [
	COV_LX, 0.0, 0.0,
	0.0, COV_LY, 0.0,
	0.0, 0.0, COV_LZ
]

COV_OX = 2.372183896856830E-06
COV_OY = 1.355307237983090E-08
COV_OZ = 4.528043672210960E-08
COV_OW = 3.742991942054960E-08

COV_ORIENTATION = [
	COV_OX, 0.0, 0.0,
	0.0, COV_OY, 0.0,
	0.0, 0.0, COV_OZ
]
