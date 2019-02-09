#!/usr/bin/env python
import roslib #roslib.load_manifest('numpy_tutorials') #not sure why I need this
import rospy
import autobot
from autobot.msg import gps_direc
from std_msgs.msg import String
import serial
import math
from math import cos, sin, atan2, sqrt


ser = serial.Serial('/dev/ttyACM1', 9600)

RAD = 3.14159265/180

def bearing_angle(lat1, long1, lat2, long2):
    long_diff = long2 * 3.14159265 / 180 - long1 * 3.14159265 / 180
    x = math.cos(lat2 * 3.14159265 / 180) * sin(long_diff)
    y = math.cos(lat1 * 3.14159265 / 180) * sin(lat2 * 3.14159265 / 180) - sin(lat1 * 3.14159265 / 180) * cos(lat2 * 3.14159265 / 180) * cos(long_diff)
    #x = sin(90 * 3.14159265 / 180);
    bearing = math.atan2(x, y)
    return bearing * 180 / 3.14159265

def distance(lat1, long1, lat2, long2):
    radius = 6371000;   #in metres
    #converting degrees to radians
    #calculating the diff of latitudes and longitudes
    long_diff = long2*3.14159265/180 - long1*3.14159265/180
    lat_diff = lat2*3.14159265/180 - lat1*3.14159265/180
    lat1 = lat1*3.14159265/180
    long1 = long1*3.14159265/180
    lat2 = lat2*3.14159265/180
    long2 = long2*3.14159265/180
    a = math.sin(lat_diff/2)*math.sin(lat_diff/2) + math.cos(lat1)*math.cos(lat2)*math.sin(long_diff/2)*math.sin(long_diff/2)
    c = 2*math.atan2(sqrt(a), math.sqrt(1-a))
    dist = radius*c
    return dist    

def talker():
    heading = -10
    #
    lat_1 = 0
    long_1 = 0
    dir_msg = gps_direc()
    while not rospy.is_shutdown():
        data= ser.readline() # I have "hi" coming from the arduino as a test run over the serial port
        #rospy.loginfo(data)
	#angle:latitude:longitude
        print("data = ", data.rstrip())
        #if (data.startswith("Lat:")):
       # print(len(data.rstrip()))
        #if data.find(":") != -1 and data.find("No fix") == -1:
        if len(data.rstrip()) >= 15:
            temp = data.rstrip().split(":")
            if (len(temp) == 3):
                #on uncommenting below line edit lat_1 = temp[1], long_1 = temp[2] and len of the temp will become 3 as the compass angle would be added
                heading = float(temp[0])
                lat_1 = float(temp[1])
                long_1 = float(temp[2])
        else:
            lat_1 = 0
            long_1 = 0
        
        lat_2 = 37.3353409
        long_2 = -121.8812953
        dist = -1
        dir_msg.velocity = 9830
        dir_msg.angle = 0;
        
        if (lat_1 != 0 and long_1 != 0 and heading != -1):
            angle = bearing_angle(lat_1, long_1, lat_2, long_2)
            dist = distance(lat_1, long_1, lat_2, long_2)   #in meters

            if angle < 0:
                angle += 360

            print("bearing angle = ", angle)
            degree = heading - angle
            
            #print("heading angle = ", heading)
            
            #dir_msg = gps_direc()
            dir_msg.angle = 0
            dir_msg.velocity = 10070 

            #if (dist < 10): # Stop if destination is within 10 mtrs
            #    dir_msg.angle = 0 # stop the motors, not able to calaculate correct angle to turn
            #    dir_msg.velocity = 9830
            print("lat :", lat_1)
            print("long :", long_1)
            print("compass :", heading)
            print("degree :", degree)
            
            if ((0 < degree and degree <= 5) or (-360 <= degree and degree < -355)):
                dir_msg.angle = 11 # go straight 3
                print("\nStraight\n")
            elif ((5 < degree and degree <= 15) or (-355 <= degree and degree < -345)):
                print("\nSL\n")
                dir_msg.angle = 0 # slight left: 5 degrees 2
            elif ((15 < degree and degree <= 60) or (-345 <= degree and degree < -300)):
                print("\nML\n")
                dir_msg.angle = -15 # mid left: 15 degrees 1
            elif ((60 < degree and degree <= 180) or (-300 <= degree and degree < -180)):
                print("\nFL\n")
                dir_msg.angle = -90 # full left: 60 degrees 0
            elif ((355 < degree and degree <= 360) or (-5 <= degree and degree < 0)):
                print("\nStraight\n")
                dir_msg.angle = 11 # go straight 3
            elif ((345 < degree and degree <= 355) or (-15 <= degree and degree < -5)):
                print("\nSR\n")
                dir_msg.angle = 22 # slight right: 5 degrees 4
            elif ((300 < degree and degree <= 345) or (-60 <= degree and degree < -15)):
                print("\nMR\n")
                dir_msg.angle = 37 # midright: 15degrees 5
            elif ((180 < degree and degree <= 300) or (-180 <= degree and degree <= -60)):
                print("\nFR\n")
                dir_msg.angle = 90 #full right: 60 degrees 6
            else:
                print("\nDonno\n")
                dir_msg.angle = 11 # stop the motors, not able to calaculate correct angle to turn
        
            if (dist < 7): # Stop if destination is within 10 mtrs
                print("\nReached\n")
                dir_msg.angle = 0 # stop the motors, not able to calaculate correct angle to turn
                dir_msg.velocity = 9830

        print("distance = ", dist)    
        pub.publish(dir_msg)
        rospy.sleep(0.1)
        #changed from  1 second to 0.1
        #check whats the periodic callback time in arduino code. after how much time GPS lat longs are sent and after how much time Compass angle is being sent??????
        #


if __name__ == '__main__':
    try:
      pub = rospy.Publisher('GPS', gps_direc, queue_size=10)
      rospy.init_node('gps')
      print("GPS node is started")
      talker()
    except rospy.ROSInterruptException:
      pass

