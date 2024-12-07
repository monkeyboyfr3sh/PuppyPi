#!/usr/bin/env python3
# coding=utf8

import sys
import math
import rospy
from std_srvs.srv import SetBool
from puppy_control.msg import Velocity, Pose, Gait

ROS_NODE_NAME = 'puppy_demo'

PuppyMove = {'x': 6, 'y': 0, 'yaw_rate': 0}
# x: Straightforward control, with the forward direction as the positive direction, measured in cm/s
# y: Lateral movement control, with the left direction as the positive direction, measured in cm/s. Currently, this feature is not available.
# yaw_rate: Turning control, with the counterclockwise direction as the positive direction, measured in rad/s

PuppyPose = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000, 'height': -10, 'x_shift': 0.5, 'stance_x': 0, 'stance_y': 0}
# stance_x: The distance each of the four legs is apart along the X-axis, measured in cm
# stance_y: The distance each of the four legs is apart along the Y-axis, measured in cm
# x_shift: The distance the four legs move along the X-axis. Smaller values lead to a more forward tilt while walking; larger values lead to a backward tilt. Adjust x_shift to balance the dog's movement, measured in cm
# height: The dog's height, measured from the toe to the thigh's rotational axis, in cm
# pitch: The pitch angle of the dog's body, measured in radians

gait = 'Trot'
# overlap_time: The time all four legs touch the ground, measured in seconds
# swing_time: The duration a single leg is off the ground, measured in seconds
# clearance_time: The time interval between the phases when the front and rear legs cross each other, measured in seconds
# z_clearance: The distance the paw needs to be raised during walking, measured in cm

if gait == 'Trot':
    GaitConfig = {'overlap_time': 0.2, 'swing_time': 0.3, 'clearance_time': 0.0, 'z_clearance': 5}
    PuppyPose['x_shift'] = -0.6
    # Trot gait: clearance_time = 0

elif gait == 'Amble':
    GaitConfig = {'overlap_time': 0.1, 'swing_time': 0.2, 'clearance_time': 0.1, 'z_clearance': 5}
    PuppyPose['x_shift'] = -0.9
    # Amble gait: 0 < clearance_time < swing_time

elif gait == 'Walk':
    GaitConfig = {'overlap_time': 0.1, 'swing_time': 0.2, 'clearance_time': 0.3, 'z_clearance': 5}
    PuppyPose['x_shift'] = -0.65
    # Walk gait: swing_time ≤ clearance_time

def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('is_shutdown')

if __name__ == '__main__':

    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)

    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    set_mark_time_srv = rospy.ServiceProxy('/puppy_control/set_mark_time', SetBool)
    # Stepping in place service

    rospy.sleep(0.2)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                         height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500)

    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
                               clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance'])
    rospy.sleep(0.2)

    PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])

    set_mark_time_srv(False)
    # If the dog continues to move slowly forward or backward while stepping in place, 
    # it is necessary to readjust the dog's center of gravity by fine-tuning PuppyPose['x_shift']

    while True:
        try:
            rospy.sleep(0.05)
            if rospy.is_shutdown():
                sys.exit(0)
        except:
            sys.exit(0)
