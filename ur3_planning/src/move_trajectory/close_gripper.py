#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
import time
import message_filters
import random


joint_names = ['pg70_finger_right_joint','pg70_finger_left_joint']

c = [0.012,0.012]

def up(move):
	cmd = JointTrajectory()
	cmd.header.stamp = rospy.Time.now()
	cmd.joint_names = joint_names
	points = []
	positions = move
	point = JointTrajectoryPoint()
	point.positions = positions
	point.time_from_start.secs = 0.0
	point.time_from_start.nsecs = 157114594
	points.append(point)
	cmd.points = points
	print 'Sent message!'
	return cmd

if __name__ == '__main__':
	rospy.init_node('demo_controller')
	cmd_publisher = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
	time.sleep(2)
	cmd_publisher.publish(up(c))