#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
import time
import message_filters
import random


joint_names = ['gripper_left_joint']

move = [0.3]


def up():
	rospy.init_node('demo_controller')
	cmd_publisher = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
	cmd = JointTrajectory()
	cmd.header.stamp = rospy.Time.now()
	cmd.joint_names = joint_names
	time.sleep(1)
	for i in range(len(move)):
		points = []
		positions = move[i]
		point = JointTrajectoryPoint()
		point.positions = positions
		point.time_from_start.secs = 0.0
		point.time_from_start.nsecs = 157114594
		points.append(point)
		cmd.points = points
		cmd_publisher.publish(cmd)
	print 'Sent message!'

if __name__ == '__main__':
	up()