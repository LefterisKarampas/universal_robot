#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
import time
import message_filters
import random


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q = [[2.2,0,-1.57,0,0,0],
    [1.5,0,-1.57,0,0,0],
    [1.5,-0.2,-1.57,0,0,0]]

def up():
    rospy.init_node('demo_controller')
    cmd_publisher = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    cmd = JointTrajectory()
    cmd.header.stamp = rospy.Time.now()
    cmd.joint_names = JOINT_NAMES
    time.sleep(1)
    for i in range(len(Q)):
        points = []
        positions = Q[i]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.secs = 0.0
        point.time_from_start.nsecs = 157114594
        points.append(point)
        cmd.points = points
        cmd_publisher.publish(cmd)
        time.sleep(3)
    print 'Sent message!'

if __name__ == '__main__':
    up()