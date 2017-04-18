#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import time
from tf.transformations import *
from tf import *
from copy import deepcopy
from math import pi, sin, cos

# Author: Zach Zweig-Vinegar
# This ROS Node publishes fake odometry msgs for the beam robot

class Follower:
	def __init__(self):
		rospy.init_node('Follower')
		# odom tester
		# self.odom_pub = rospy.Publisher('/beam/odom', Odometry, queue_size=1)

		self.tfl = tf.TransformListener()

		# self.FIXED_FRAME = "map"
		self.FIXED_FRAME = "odom"
		self.BASE_FRAME = "base_link"

		# publishes fake odometry at 10 Hz
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			time = rospy.Time.now()
			rospy.loginfo("Iteration")
			rate.sleep()

		rospy.spin()

	# transform the pose stamped to the new frame
	def transform_pose(self, new_frame, ps):
		if ps.header.frame_id == new_frame:
			return ps
		try:
			temp_ps = deepcopy(ps)
			temp_ps.header.stamp = rospy.Time(0)
			self.tfl.waitForTransform(temp_ps.header.frame_id, new_frame, rospy.Time(0), rospy.Duration(4.0))
			new_pose = self.tfl.transformPose(new_frame, temp_ps)
			new_pose.header.stamp = deepcopy(ps.header.stamp)
			return new_pose
		except Exception as e:
			rospy.logerr(e)
			rospy.logerr("no transform")
			return None

	def gmapBaseTo(self, ps):
		goal = MoveBaseGoal()
		self.move_base_target = self.transform_pose("map", ps)
		goal.target_pose = self.move_base_target
		rospy.logerr(goal)
		self.move_base.send_goal(goal)

	def cancelgmapBaseTo(self) :
		rospy.logerr("Cancelling current action")
		self.move_base.cancel_goal()

	def wait_for_move_base(self):
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

		if not success:
				self.move_base.cancel_goal()
				rospy.logerr("move_base FAILURE")
		else:
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo("move_base SUCCEEDED")

if __name__ == '__main__':
	Follower()