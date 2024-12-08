#!/usr/bin/python3
#coding=utf8
# Chapter 8: ROS Robot Expansion Course\2. Raspberry Pi Expansion Board Course\Lesson 2 Control RGB Color Light
import os
import sys
import rospy
from std_msgs.msg import *
from ros_robot_controller.msg import RGBState, RGBsState


print('''
**********************************************************
****************Function: RGB Color Light Control Routine***********************
**********************************************************
----------------------------------------------------------
Official website: https://www.hiwonder.com
Online mall: https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program; if it fails, please try multiple times!
----------------------------------------------------------
''')

# Turn off RGB color light
def turn_off_rgb():
    led1 = RGBState()
    led1.r = 0
    led1.g = 0
    led1.b = 0
    led1.id = 1

    led2 = RGBState()
    led2.r = 0
    led2.g = 0
    led2.b = 0
    led2.id = 2
    msg = RGBsState()
    msg.data = [led1, led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)

# Set the display of RGB color light
def set_rgb_show(r, g, b):
    led1 = RGBState()
    led1.r = r
    led1.g = g
    led1.b = b
    led1.id = 1

    led2 = RGBState()
    led2.r = r
    led2.g = g
    led2.b = b
    led2.id = 2
    msg = RGBsState()
    msg.data = [led1, led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)

# Close detection function
run_st = True
def Stop():
    global run_st
    run_st = False
    turn_off_rgb()
    print('Shutting down...')


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('rgb_control_demo')
    rospy.on_shutdown(Stop)
   
    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    rospy.sleep(0.2)  # Delay for a moment to wait for the subscription to take effect
    
    while run_st:
        r, g, b = 0, 0, 0
        for r in range(0, 255, 5):  # The red color gradually brightens
            set_rgb_show(r, g, b)
            rospy.sleep(0.005)
            
        rospy.sleep(1)
        r, g, b = 0, 0, 0 
        for g in range(0, 255, 5):  # The green color gradually brightens
            set_rgb_show(r, g, b)
            rospy.sleep(0.005)
        
        rospy.sleep(1)
        r, g, b = 0, 0, 0 
        for b in range(0, 255, 5):  # The blue color gradually brightens
            set_rgb_show(r, g, b)
            rospy.sleep(0.005)
        rospy.sleep(1)
