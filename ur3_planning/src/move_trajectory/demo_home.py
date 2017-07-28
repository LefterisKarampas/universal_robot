#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
import time
import message_filters
import random


joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

home = [-6.828689300455153e-05, -8.604538985528053e-05, -9.053833968937402e-06, 8.107810746878387e-07, 8.790735388174647e-06, 6.346890036948025e-05]

def init():
	rospy.init_node('demo_controller')
	cmd_publisher = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
	cmd = JointTrajectory()
	time.sleep(1)
	cmd.header.stamp = rospy.Time.now()
	cmd.joint_names = joint_names
	points = []
	positions = home
	point = JointTrajectoryPoint()
	point.positions = positions
	point.time_from_start.secs = 0.0
	point.time_from_start.nsecs = 157114594
	points.append(point)
	cmd.points = points
	cmd_publisher.publish(cmd)
	print 'Sent message!'



if __name__ == '__main__':
	init()