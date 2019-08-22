#!/usr/bin/env python
 
import rospy
import roslib
import numpy
import math
import tf
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry

import time
import threading

import sys
import serial

import parameter
import controller


arming_flag = False
exit_flag = False

rs485_lock = threading.Lock()
command_lock = threading.Lock()
control_lock = threading.Lock()

vel_spatial = 0.0
vel_angular = 0.0
vel_fan = 0.0

sDrvFan = serial.Serial(
    port     = parameter.SER_DRV_FAN,
    baudrate = parameter.SER_BAUDRATE,
    parity   =    serial.PARITY_NONE,
    stopbits =    serial.STOPBITS_ONE,
    bytesize =    serial.EIGHTBITS,
    timeout  = parameter.SER_TIMEOUT
)

cmdSubscriber = None
fanSubscriber = None

if sDrvFan.isOpen():
    print("Succeeded to connect to drive fan controller")
else:
    print("Failed to connect to the drive fan controller")
    utils.getch()
    quit()


def cmd_vel_callback(data):
    global vel_spatial
    global vel_angular
    # rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.linear.x)
    vel_spatial = data.linear.x
    vel_angular = data.angular.z


def cmd_vel_listener():
    global cmdSubscriber
    cmdSubscriber = rospy.Subscriber("cmd_vel", Twist, callback=cmd_vel_callback, queue_size=10)
    # rospy.init_node('odom_node', anonymous=True)
    # odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    # rospy.spin()

def fan_vel_callback(data):
    global vel_fan
    vel_fan = data

def fan_vel_listener():
    global fanSubscriber
    fanSubscriber = rospy.Subscriber("fan_vel", Float32, callback=fan_vel_callback, queue_size=3)


def init_listener():
    rospy.init_node('motor_controllers', anonymous=False)
    cmd_vel_listener()
    fan_listener()

def stop_listener():
    if None != cmdSubscriber:
        cmdSubscriber.unregister()
        cmdSubscriber = None
    if None != fanSubscriber:
        fanSubscriber.unregister()
        fanSubscriber = None



class Subscriber(threading.Thread):
    '''
    subscribe and read the motor's state data
    '''

    def __init__(self, threadID, name, controller, time_step):
        '''
        set up the subscriber
        arguments:
            int [thread ID]
            str [thread name]
            command.controller [target controller]
            float [time step]
        '''

        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.controller = controller
        self.time_step = time_step

    def run(self):
        '''
        run the subscriber thread
        returns:
            bool [terhimation without an error]
        '''

        while arming_flag:
            rs485_lock.acquire()
            self.controller.request()
            rs485_lock.release()

            time.sleep(self.time_step)

        printout = 'Thread ' + str(self.threadID) + ' disabled\r\n'
        sys.stdout.write(printout)
        return True


class Publisher(threading.Thread):
    '''
    publish the motor's state data
    '''

    def __init__(self, threadID, name, controller, time_step):
        '''
        set up the publisher
        arguments:
            int [thread ID]
            str [thread name]
            command.controller [target controller]
            float [time step]
        '''

        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.time_step = max(time_step - 0.05, 0)
        self.controller = controller

    def run(self):
        '''
        run the publisher thread
        returns:
            bool [terhimation without an error]
        '''

        while arming_flag:
            rs485_lock.acquire()
            control_lock.acquire()
            data = self.controller.control()
            control_lock.release()
            time.sleep(0.05)  # 0.005 with onboard
            rs485_lock.release()

            time.sleep(self.time_step)

        printout = 'Thread ' + str(self.threadID) + ' disabled\r\n'
        sys.stdout.write(printout)
        return True


class Key(threading.Thread):

    def __init__(self, time_step):
        threading.Thread.__init__(self)
        self.threadID = 0
        self.name = "emergency_key"
        self.time_step = time_step
        self.time_init = time.time()
        self.vel_spatial = 0
        self.vel_angular = 0

    def run(self):
        '''
        run the subscriber thread
        returns:
                bool [terhimation without an error]
        '''

        global arming_flag

        while arming_flag:
            # time_past = time.time()- self.time_init
            # if time_past >10:
            # 	arming_flag = False
            ch = command.getch()

            if ch == 'q':
                arming_flag = False
                break

            time.sleep(self.time_step)

        printout = 'Thread ' + str(self.threadID) + ' disabled\r\n'
        sys.stdout.write(printout)
        return True


if __name__ == "__main__":

    queue = []

    try:
        port = serial.Serial(
            port     = parameter.SER_DRV_FAN,
            baudrate = parameter.SER_BAUDRATE,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE,
            bytesize = serial.EIGHTBITS,
            timeout  = parameter.SER_TIMEOUT
        )
        driving_controller = controller.Controller(port, controller.TYPE_DEV_DRV)
        fan_controller = controller.Controller(port, controller.TYPE_DEV_FAN)
        init_listener()
        set_key = key(0.005)

        # observer_thread = Subscriber(1, "observer", driving_controller, 0.5)
        controller_thread = Publisher(2, "controller", driving_controller, 0.5)
        fan_controller_thread = Publisher(3, "controller", fan_controller, 0.7)

        # queue.append(observer_thread)
        queue.append(controller_thread)
        queue.append(fan_controller_thread)

        try:

            arming_flag = True
            set_key.start()

            driving_controller.enable()
            fan_controller.enable()
            sys.stdout.write('Motor controller enabled\r\n')
            time.sleep(5.0e-1)

            # set default value

            driving_controller.control()
            time.sleep(1.0e-2)

            # observer_thread.start()
            controller_thread.start()
            time.sleep(1.0e-2)
            fan_controller_thread.start()

            time_init = time.time()

            time.sleep(1.0)

            control_lock.acquire()
            driving_controller.refer((0, 0))
            fan_controller.refer((0, 0))
            control_lock.release()

            driving_controller.control()
            time.sleep(5.0e-2)
            
            fan_controller.control()
            time.sleep(5.0e-2)

            while True:
                time_past = time.time() - time_init

                command_lock.acquire()
                left_val = int(300*(-vel_spatial - vel_angular))
                right_val = int(300*(vel_spatial - vel_angular))
                command_lock.release()

                control_lock.acquire()
                driving_controller.refer((left_val, right_val))
                control_lock.release()

                time.sleep(5.0e-2)

                command_lock.acquire()
                if 0.0 <= vel_fan:
                    fan_val = 0
                elif 1.0 <= vel_fan:
                    fan_val = parameter.FAN_MAX + parameter.FAN_OFFSET
                else:
                    fan_val = int((parameter.FAN_MAX - parameter.FAN_MIN) * fan_val + parameter.FAN_MIN + parameter.FAN_OFFSET)
                command_lock.release()

                control_lock.acquire()
                fan_controller.refer((fan_val, 0))
                control_lock.release()

                if arming_flag != True:
                    for s in queue:
                        s.join

                    sys.stdout.write("Exiting main thread\r\n")

                    control_lock.acquire()
                    driving_controller.refer((0, 0))
                    control_lock.release()

                    driving_controller.control()
                    time.sleep(5.0e-2)

                    driving_controller.disable()
                    sys.stdout.write("Motor controller disabled\r\n")
                    time.sleep(1.0e-1)

                    break

        except:
            print ("Error: unable to start thread")
        
        stop_listener()

    except:
        print ("Error: unable to generate thread")