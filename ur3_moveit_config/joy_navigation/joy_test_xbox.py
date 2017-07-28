 
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

def callback(states,joy_move):
	cmd_publisher = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
	for i in range(len(list(joy_move.buttons))):
		if joy_move.buttons[i] == 1:
			break
	cmd = JointTrajectory()
	cmd.header.stamp = rospy.Time.now()
	cmd.joint_names = joint_names
	points = []
	positions = list(states.position)
	point = JointTrajectoryPoint()
	factor = 1
	j = -1
	if i < 6:
		if i == 0:
			j = 4
		elif i == 1:
			j = 3
		elif i == 2:
			j = 5
		elif i == 3:
			j = 2
		elif i == 4:
			j = 0
		else:
			j = 1 
		temp = list(joy_move.axes)
		positions[j] += temp[3] * factor *0.2
		point.positions = positions
		point.time_from_start.secs = 0.0
		point.time_from_start.nsecs = 157114594
		points.append(point)
		cmd.points = points
		cmd_publisher.publish(cmd)

def init():
	rospy.init_node('safety_controller')
	joy_subscriber = message_filters.Subscriber('/joy',Joy)
	joint_state_subscriber = message_filters.Subscriber('/joint_states',JointState)
	ts = message_filters.ApproximateTimeSynchronizer([joint_state_subscriber,joy_subscriber], 10, 0.1, allow_headerless=True)
	ts.registerCallback(callback)
	rospy.spin()
	print 'Sent message!'



if __name__ == '__main__':
	init()