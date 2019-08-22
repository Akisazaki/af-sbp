import parameter
import utils

import serial
import math

import termios
import tty
import sys


TYPE_DEV_DRV = 0
TYPE_DEV_FAN = 1


def pack(msg_function, tup_val):
	'''
	generate a command from a target ID and messages
	arguments:
		int [target ID], str [command message]
	returns:
		str [command]
	'''

	cmd_out = parameter.MSG_START

	cmd_out += msg_function

	for idx in range(0, parameter.MSG_DEV_NUMBER):
		cmd_out += '%0*d' % (parameter.MSG_RESOLUTION, tup_val[idx])

	cmd_out += parameter.MSG_END

	cmd_out += '\n'

	return str.encode(cmd_out)


def unpack(str_message):
	'''
	generate a command from a target ID and messages
	arguments:
		int [target ID], str [command message]
	returns:
		str [command]
	'''

	tup_val = [0, 0]

	if str_message[0] == parameter.MSG_START and str_message[-1] == parameter.MSG_END:

		idx = 2

		for subidx in range(0, parameter.MSG_DEV_NUMBER):
			for ssubidx in range(0, parameter.MSG_RESOLUTION):
				tup_val[subidx] = 10 * tup_val[0]
				tup_val[subidx] += int(str_message[idx], 10)
				idx += 1

	return tup_val


class Controller:
    """
    Controller for Drive and Fan
    """

    def __init__(self, port, dev_type):
        '''
        init motor controller
        arguments:
            Serial [port],
            int [dev_type]
        returns:
            Controller
        '''
        self.port = port
        if dev_type == TYPE_DEV_DRV:
            self.key_enable = parameter.MSG_DRV_ENABLE
            self.key_control = parameter.MSG_DRV_CONTROL
            self.key_request = parameter.MSG_DRV_REQUEST
            self.val_offset = parameter.DRV_OFFSET
            self.val_max = parameter.DRV_MAX
            self.val_min = parameter.DRV_MIN
        elif dev_type == TYPE_DEV_FAN:
            self.key_enable = parameter.MSG_FAN_ENABLE
            self.key_control = parameter.MSG_FAN_CONTROL
            self.key_request = parameter.MSG_FAN_REQUEST
            self.val_offset = parameter.FAN_OFFSET
            self.val_max = parameter.FAN_MAX
            self.val_min = parameter.FAN_MIN

    def enable(self):
        '''
        enable this motor controller
        returns:
            bool [success]
        '''
        self.str_command = pack(self.key_enable, (1, 1))
        if self.port.isOpen():
            self.port.write(self.str_command)
            self.str_command = ''
            return True
        else:
            return False

    def disable(self):
        '''
        disable this motor controller
        returns:
            bool [success]
        '''
        self.str_command = self.pack(self.key_enable, (0, 0))
        if self.port.isOpen():
            self.port.write(self.str_command)
            self.str_command = ''
            return True
        else:
            return False

    def refer(self, tup_input):
        '''
        Set the motor's controller setpoint
        returns:
            bool [success]
        '''
        self.tup_reference = tup_input
        return True

    def feedback(self):
        '''
        Save feedback value from motor
        returns:
            bool [success]
        '''
        self.tup_feedback = unpack(self.str_received)
        return True
    
    def control(self):
        '''
        Control the motors with respec to the setpoint
        returns:
            bool [success]
        '''
        self.str_command = pack(self.key_control, self.tup_reference)
        if self.port.isOpen():
            self.port.write(self.str_command)
            return True
        else:
            return False
            
    def request(self):
        '''
        request the motors' state data and receive raw messages
        returns:
            bool [success]
        '''
        self.str_command = pack(self.key_request, (0, 0))
        if self.port.isOpen():
            self.port.write(self.str_command)
            self.str_command = ''
            # out = ''
            # out = SER.read(parameter.MSG_LENGTH)
            # if out !='':
            #   self.srt_received = out
            #   return True
        else:
            return False
