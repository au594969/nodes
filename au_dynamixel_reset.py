#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
import math

def invkin(xyz):
	
	q1 = 0
	q2 = 0
	q3 = 0
	q4 = 0

	print(q1,-q2,q3,q4)
	return q1,-q2,q3,q4

class ActionExampleNode:

	N_JOINTS = 4
	def __init__(self,server_name):
		self.client = actionlib.SimpleActionClient(server_name, FollowJointTrajectoryAction)

		self.joint_positions = []
		self.names =["joint1",
				"joint2",
				"joint3",
				"joint4"
				]
		
#Trial test

		xarr = [x for x in range(-30,31,1)]

		yarr = [-20]*len(xarr)

		zarr = [5]*len(xarr)



# the list of xyz points we want to plan
		#xyz_positions = [
		#[0.2, 0.0, 0.2],
		#[0.25, 0.1, 0.2],
		#[0.2, 0.2, 0.2]
		#]
		# initial duration
		dur = rospy.Duration(1)

		# construct a list of joint positions by calling invkin for each xyz point
		for p in range(61):
			jtp = JointTrajectoryPoint(positions=invkin([xarr[p],yarr[p],zarr[p]]),velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
			dur += rospy.Duration(0.2)
			self.joint_positions.append(jtp)

		self.jt = JointTrajectory(joint_names=self.names, points=self.joint_positions)
		self.goal = FollowJointTrajectoryGoal( trajectory=self.jt, goal_time_tolerance=dur+rospy.Duration(2) )

	def send_command(self):
		self.client.wait_for_server()
		print(self.goal)
		self.client.send_goal(self.goal)

		self.client.wait_for_result()
		print(self.client.get_result())

if __name__ == "__main__":
	rospy.init_node("au_dynamixel_invkin_reset")

	node= ActionExampleNode("/arm_controller/follow_joint_trajectory")

	node.send_command()
