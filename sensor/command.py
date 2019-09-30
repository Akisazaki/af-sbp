# dust: 132.4
#   td1324q
# sonar: 9.543
#   ts9543q

#!/usr/bin/python

'''
Project:
	Smart Mobile Air Purifier (SMAP) Prototype

Title:
	Mobile Platform Command Functions

Description
	-generate a control input command from desired state
	-set and change a motor controller's state
	-set and publish a control input command
	-request and read state data

* Copyrighted by Mingyo Seo
* Created on October 13 2018
'''

import math

import time

import sys
import tty
import termios
import serial
import parameter


# configure the serial connections "/dev/tty.usbserial-A505ABAD 115200 8N1"
SER = serial.Serial(
    port     = parameter.SER_DEVICE,
    baudrate = parameter.SER_BAUDRATE,
    parity   = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout  = parameter.SER_TIMEOUT
)

positive_inf = float("inf")
negative_inf = float("-inf")


if SER.isOpen():
	print("Succeeded to conncte to the sensor controller")
else:
	print("Failed to conncte to the sensor controller")
	print("Press any key to terminate...")
	getch()
	quit()


def getch():

	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)

	try:
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)
		print(ch)
	except:
		print("error")
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

	return ch


def unpack(msg, start, length):
	'''
	unpack string as number
	arguments:
		string [msg], int [start], int [length]
	returns:
		int [number]
	'''
	token = msg[start:(start+length)]
	value = int(token)
	# for index in range(start, start + length):
	# 	value = value * 10
	# 	value += int(msg[index], 10)
	return value



class controller:
	'''
	set a controller's configuration and execute commands
	'''

	def __init__(self, imuCallback, rangeCallback, dustCallback, inaCallback):
		'''
		configure the controller's ID and control mode
		arguments:
			imuCallback [void (qx,qy,qz,qw, gx,gy,gz, ax,ay,az)]
			rangeCallback [void (type, id, range)]
			dustCallback [void (id, dust_level)]
			inaCallback [void (voltage, current, power)]
		'''
		self.imuCallback = imuCallback
		self.rangeCallback = rangeCallback
		self.dustCallback = dustCallback
		self.inaCallback = inaCallback
		
		self.str_command = ''
		self.srt_received = ""

		self.sonar = 0.0
		self.sonar1 = 0.0
		self.dust = 0.0


	def enable(self):
		'''
		enable the sensor controller
		returns:
			bool [success]
		'''

		# self.str_command = pack(parameter.MSG_ENABLE, (1, 1))

		if SER.isOpen():
			# SER.write(self.str_command)
			# print(self.str_command)
			# self.str_command = ''
			return True
		else:
			return False


	def disable(self):
		'''
		disable the sensor controller
		returns:
			bool [success]
		'''

		# self.str_command = pack(parameter.MSG_ENABLE, (0, 0))

		self.srt_received = ""

		if SER.isOpen():
			# SER.write(self.str_command)
			return True
		else:
			return False

	def readNumber(self):
		self.srt_received = ""
		while SER.isOpen():
			ch = SER.read()
			ch.strip()
			if len(ch) > 0:
				if ch =='+' or ch =='-' or ch == '.' or ch.isdigit():
					self.srt_received += ch
				else:
					break
		if len(self.srt_received) > 0:
			return float(self.srt_received)
		else:
			return 0.0

	def readIMU(self):
		self.imuCallback(
			self.readNumber(),self.readNumber(),self.readNumber(),self.readNumber(), 
			self.readNumber(),self.readNumber(),self.readNumber(), 
			self.readNumber(),self.readNumber(),self.readNumber())
	
	def readINA(self):
		self.inaCallback(self.readNumber(), self.readNumber(), self.readNumber())

	def readLine(self):
		'''
		read serial input
		auto detect MSG_START and MSG_END
		if input length is bigger then twice of MSG_LENGTH will return and finish.
		'''
		while SER.isOpen():
			try:
				ch = SER.read()
				ch.strip()
				if len(ch) > 0:
					if ch == parameter.MSG_IMU:
						self.readIMU()
					elif ch == parameter.MSG_INA
						self.readINA()
					elif ch == parameter.MSG_START:
						self.srt_received = ch
					elif ch == parameter.MSG_END:
						out = self.srt_received + ch
						self.srt_received = ""
						return out
					else:
						length = len(self.srt_received)
						if length > parameter.MSG_LENGTH * 2:
							out = self.srt_received
							self.srt_received = ""
							return out
						elif length > 0:
							self.srt_received += ch
			except ValueError as ve:
				print "Ignore uncompletable input %s, error: %s" % (self.srt_received, ve)
				self.srt_received = ""
			time.sleep(0.001)

	# def readLine(self):
	# 	'''
	# 	read serial input
	# 	auto detect MSG_START and MSG_END
	# 	if input length is bigger then twice of MSG_LENGTH will return and finish.
	# 	'''
	# 	while SER.isOpen():
	# 		ch = SER.read()
	# 		ch.strip()
	# 		if ch == parameter.MSG_WIFI_ENABLE:
	# 			print('Do something related wifi thing here')
	# 		elif ch == parameter.MSG_WIFI_START:
	# 			self.srt_received = ch
	# 			while True:
	# 				if ch == parameter.MSG_WIFI_END:
	# 					out = self.srt_received + ch
	# 					self.srt_received = ""
	# 					print(out)
	# 					return out
	# 				else:
	# 					length = len(self.srt_received)
	# 					if length > parameter.MSG_WIFI_MAXIMUM:
	# 						out = ""
	# 						self.srt_received = ""
	# 						return out
	# 					else:
	# 						self.srt_received += ch
	# 		elif ch == parameter.MSG_START:
	# 			self.srt_received = ch
	# 		elif ch == parameter.MSG_END:
	# 			out = self.srt_received + ch
	# 			self.srt_received = ""
	# 			return out
	# 		else:
	# 			length = len(self.srt_received)
	# 			if length > parameter.MSG_LENGTH * 2:
	# 				out = self.srt_received
	# 				self.srt_received = ""
	# 				return out
	# 			else:
	# 				self.srt_received += ch
	# 		time.sleep(0.001)


	def receive(self):
		'''
		receive the sensor's data and receive raw messages
		returns:
			bool [success]
		'''

		# self.str_command = pack(parameter.MSG_REQUEST, (0, 0))

		if SER.isOpen():
			# SER.write(self.str_command)
			# self.str_command = ''
			# out = ''

			out = self.readLine()
			length = len(out)
			if len(out) == parameter.MSG_LENGTH and out[0] == parameter.MSG_START and out[-1] == parameter.MSG_END:
				# sys.stdout.write("{0}\r\n".format(out))
				# dust0: 132.4
				#   td0|1324q
				# sonar1: 9.543
				#   ts1|9543q
				# sonar: +inf
				#   ts0|9999q
				# sonar: -inf
				#   ts0|0000q
				sensor_type = out[1]
				sensor_number = int(out[2])
				if parameter.MSG_SONAR == sensor_type or parameter.MSG_IR == sensor_type:
					val = unpack(out, 4, 4)
					if 0 == val:
						val = negative_inf # in python 3 -math.inf
					elif 9999 == val:
						val = positive_inf # in python 3 math.inf
					else:	
						val = val * 0.001
					
					self.rangeCallback(sensor_type, sensor_number, val)

					if sensor_number == 0:
						self.sonar = val
					elif sensor_number == 1:
						self.sonar1 = val
					return False

				elif parameter.MSG_DUST == sensor_type:
					val = unpack(out, 4, 4)
					self.dust = val * 0.1
					self.dustCallback(0, val)
					return True
		
		return False