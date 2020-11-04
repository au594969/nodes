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
	"""
	Python implementation of the the inverse kinematics for the crustcrawler
	Input: xyz position
	Output: Angels for each joint: q1,q2,q3,q4
	
	You might adjust parameters (d1,a1,a2,d4).
	The robot model shown in rviz can be adjusted accordingly by editing 		au_crustcrawler_ax12.urdf
	"""

	d1 = 8.57 # cm (height of 2nd joint)
	a1 = 0.0 # (distance along "y-axis" to 2nd joint)
	a2 = 17.2 # (distance between 2nd and 3rd joints)
	d4 = 23.5 # (distance from 3rd joint to gripper center - all inclusive, ie. also 4th joint)

	# Calculate oc
	oc = xyz # - d4*R*[0;0;1]
	xc = oc[0]
	yc = oc[1]
	zc = oc[2]
	
	
	# Calculate q1
	q1 = math.atan2(yc, xc)  # NOTE: Order of y and x.. depend on math.atan2 func..
	
	
	# Calculate q2 and q3
	r2 = (xc - a1*math.cos(q1))**2 + (yc - a1*math.sin(q1))**2 # radius squared - radius can never be negative, q1 accounts for this..
	s = (zc - d1) # can be negative ! (below first joint height..)
	D = ( r2 + s**2 - a2**2 - d4**2)/(2*a2*d4)   # Eqn. (3.44)
	
	q3 = math.atan2(-math.sqrt(1-D**2), D) #  Eqn. (3.46)
	q2 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4*math.sin(q3), a2 + d4*math.cos(q3)) # Eqn. (3.47)
	
	
	# calculate q4 - ie. rotation part
	# r32 = R(3,2);
	# r23 = R(2,3);
	# c23 = math.cos(q2 + q3);
	# q4 = math.atan2(r23/c23, r32/c23); 
	q4 = 0 # not consider rotation so far..

	return q1,q2,q3,q4

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
		# the list of xyz points we want to plan
		xyz_positions = [
		[0.2, 0.0, 0.2],
		[0.25, 0.1, 0.2],
		[0.2, 0.2, 0.2]
		]
		# initial duration
		dur = rospy.Duration(1)

		# construct a list of joint positions by calling invkin for each xyz point
		for p in xyz_positions:
			jtp = JointTrajectoryPoint(positions=invkin(p),velocities=[0.5]*self.N_JOINTS ,time_from_start=dur)
			dur += rospy.Duration(2)
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
	rospy.init_node("au_dynamixel_test_node")

	node= ActionExampleNode("/arm_controller/follow_joint_trajectory")

	node.send_command()
