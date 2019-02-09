#!/usr/bin/env python
import roslib #roslib.load_manifest('numpy_tutorials') #not sure why I need this
import rospy
import autobot
from autobot.msg import drive_pwm
from std_msgs.msg import String
import serial
import math
from math import cos, sin, atan2, sqrt


ser = serial.Serial('/dev/ttyACM1', 9600)


def callback(data):
    ser.write(data.pwm_drive)
    ser.write(data.pwm_angle)
      

def talker():
    rospy.Subscriber("drive_pwm", drive_pwm, callback)
    rospy.spin()


if __name__ == '__main__':
    try:
      rospy.init_node('ard_mtr')
      talker()
    except rospy.ROSInterruptException:
      pass

