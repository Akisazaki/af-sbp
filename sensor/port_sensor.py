#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import Quaternion, Vector3

import time
import threading

import sys
import tty
import termios
import serial
import math
import parameter

import command
import parameter

arming_flag = False

rs485_lock = threading.Lock()

sonar_seq = 0
sonar1_seq = 0
sonar_pub = None
sonar1_pub = None
dust_pub  = None
imu_seq = 0
imu_pub = None


def listener():
    global sonar_pub
    global sonar1_pub
    global dust_pub
    global imu_pub
    rospy.init_node('sensor_node', anonymous=True)
    sonar_pub = rospy.Publisher('sonar', Range, queue_size=1)
    sonar1_pub = rospy.Publisher('sonar1', Range, queue_size=1)
    dust_pub = rospy.Publisher('dust', Float32, queue_size=1)
    imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
    # rospy.spin()

def dispose():
    global sonar_pub
    global sonar1_pub
    global dust_pub
    global imu_pub
    if None != sonar_pub:
        sonar_pub.unregister()
        sonar_pub = None
    if None != sonar1_pub:
        sonar1_pub.unregister()
        sonar1_pub = None
    if None != dust_pub:
        dust_pub.unregister()
        dust_pub = None
    if None != imu_pub:
        imu_pub.unregister()
        imu_pub = None


class subscriber(threading.Thread):
    '''
    subscribe and read the sensor's state data
    '''

    def __init__(self, threadID, name, controller, time_step, on_receive):
        '''
        set up the subscriber
        arguments:
                int [thread ID], str [thread name], command.controller [target controller], float [time step]
        '''

        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.controller = controller
        self.time_step = time_step
        self.on_receive = on_receive
        self.sonar = 0.0
        self.sonar1 = 0.0
        self.dust = 0.0

    def run(self):
        '''
        run the subscriber thread
        returns:
                bool [terhimation without an error]
        '''

        while arming_flag:
            rs485_lock.acquire()
            if self.controller.receive():
                self.dust = self.controller.dust
                self.sonar = self.controller.sonar
                self.sonar1 = self.controller.sonar1
                self.on_receive(self)
            rs485_lock.release()

            time.sleep(self.time_step)

        printout = 'Thread ' + str(self.threadID) + ' disabled\r\n'
        sys.stdout.write(printout)
        return True


class key(threading.Thread):

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
            ch = command.getch()

            if ch == 'q':
                arming_flag = False
                break

            time.sleep(self.time_step)

        printout = 'Thread ' + str(self.threadID) + ' disabled\r\n'
        sys.stdout.write(printout)
        return True


def on_imu(qx,qy,qz,qw, gx,gy,gz, ax,ay,az):
    global imu_pub
    global imu_seq

    imu_seq += 1
    if imu_seq < parameter.IMU_IGNORE_COUNT:
        return

    msg = Imu()
    
    msg.header.seq = imu_seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = parameter.IMU_FRAME_ID

    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw

    msg.angular_velocity.x = gx
    msg.angular_velocity.y = gy
    msg.angular_velocity.z = gz

    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az

    msg.angular_velocity_covariance = parameter.COV_ANGULAR
    msg.linear_acceleration_covariance = parameter.COV_LINEAR
    msg.orientation_covariance = parameter.COV_ORIENTATION

    imu_pub.publish(msg)

def on_sonar(id, range):
    global sonar_pub
    global sonar1_pub
    global sonar_seq
    global sonar1_seq
    
    if math.isinf(range) or math.isnan(range):
        return
    
    msg = Range()
    msg.header.stamp = rospy.Time.now()
    msg.range = range
    fieldOfViewTable = parameter.SONAR_T_FIELD_OF_VIEWS
    if id == 0:
        sonar_seq += 1
        msg.header.seq = sonar_seq
        msg.header.frame_id = parameter.SONAR_T_FRAME_ID
        # msg.field_of_view = parameter.SONAR_T_FIELD_OF_VIEW
        msg.min_range = parameter.SONAR_T_MIN_RANGE
        msg.max_range = parameter.SONAR_T_MAX_RANGE

    else:
        fieldOfViewTable = parameter.SONAR_B_FIELD_OF_VIEWS
        sonar1_seq += 1
        msg.header.seq = sonar1_seq
        msg.header.frame_id = parameter.SONAR_B_FRAME_ID
        # msg.field_of_view = parameter.SONAR_B_FIELD_OF_VIEW
        msg.min_range = parameter.SONAR_B_MIN_RANGE
        msg.max_range = parameter.SONAR_B_MAX_RANGE
    
    weight = range * 0.1
    index = int(round(weight, 0))
    if index >= len(fieldOfViewTable) - 1:
        msg.field_of_view = fieldOfViewTable[len(fieldOfViewTable) - 1]
    else:
        weight = weight % 1.0
        msg.field_of_view = fieldOfViewTable[index] * (1.0 - weight) + fieldOfViewTable[index + 1] * weight
    if id == 0:
        sonar_pub.publish(msg)
    else:
        sonar1_pub.publish(msg)

def on_dust(id, value):
    global dust_pub
    dust = Float32()
    dust.data = value
    dust_pub.publish(dust)

def _on_receive(subscriber):
    global sonar_pub
    global sonar_seq
    #sys.stdout.write('Sensor Value\r\n Sonar0: {0}\r\n Sonar1: {1}\r\n Dust: {2}\r\n'.format(subscriber.sonar, subscriber.sonar1, subscriber.dust))
    timestamp = rospy.Time.now()
    sonar_seq += 1
    msg = Range()
    msg.header.seq = sonar_seq
    msg.header.stamp = timestamp
    msg.header.frame_id = parameter.SONAR_FRAME_ID
    # msg.ULTRASOUND = 1
    # msg.INFRARED = 0
    msg.field_of_view = parameter.SONAR_FIELD_OF_VIEW
    msg.min_range = parameter.SONAR_MIN_RANGE
    msg.max_range = parameter.SONAR_MAX_RANGE
    msg.range = subscriber.sonar
    sonar_pub.publish(msg)

    msg = Range()
    msg.header.seq = sonar_seq
    msg.header.stamp = timestamp
    msg.header.frame_id = parameter.SONAR_FRAME_ID
    # msg.ULTRASOUND = 1
    # msg.INFRARED = 0
    msg.field_of_view = parameter.SONAR_FIELD_OF_VIEW
    msg.min_range = parameter.SONAR_MIN_RANGE
    msg.max_range = parameter.SONAR_MAX_RANGE
    msg.range = subscriber.sonar1
    sonar1_pub.publish(msg)

def on_receive(subscriber):
    global dust_pub
    dust_pub.publish(subscriber.dust)


if __name__ == "__main__":

    queue = []

    try:
        listener()
        set_key = key(0.005)

        sensor_controller = command.controller(imuCallback=on_imu, sonarCallback=on_sonar, dustCallback=on_dust)
        # observer_thread = subscriber(1, "observer", sensor_controller, 0.05, on_receive)
        # queue.append(observer_thread)

        try:
            arming_flag = True
            set_key.start()

            sensor_controller.enable()
            sys.stdout.write('Sensor controller enabled\r\n')
            time.sleep(5.0e-1)

            # observer_thread.start()

            time_init = time.time()

            time.sleep(1.0)

            while arming_flag and not rospy.is_shutdown():
                # time.sleep(1.0e-1)
                sensor_controller.receive()
                time.sleep(1.0e-5)
            
            for s in queue:
                s.join

            sys.stdout.write("Exiting main thread\r\n")

            sensor_controller.disable()
            sys.stdout.write("Sensor controller disabled\r\n")
            time.sleep(1.0e-1)

        except Exception as e1:
            print ("Error: unable to start thread")
            print e1
        
        dispose()

    except Exception as e2:
        print ("Error: unable to generate thread")
        print e2
