#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped, Quaternion, PoseStamped
import scipy
import time
import tf # for tf.transformations.quaternion_from_euler(*self.curr_j6_pose[3:])
import numpy as np
from std_msgs.msg import Float64MultiArray
import math

class SubjectSimPublisher:
	def __init__(self):
		self.pub_topic = "/sensor_values"
		self.top_type = WrenchStamped
		self.pub = rospy.Publisher(self.pub_topic, self.top_type, queue_size=1)
		self.msg = self.top_type()
		rospy.init_node('SimSubject', anonymous=False)
		self.hz = 500
		self.dt = 1.0 / self.hz
		self.rate = rospy.Rate(self.hz)
		self.position_tol = 0.01
		self.orientation_tol = 0.2

		self.fk_sub = rospy.Subscriber("/iiwa/j6_pose_custom", Float64MultiArray, self.updateCurrentPose)
		self.target_sub = rospy.Subscriber("/CurrentTargets", Float64MultiArray, self.updateCurrentTarget)
		self.initial_pose = np.array([0.0, -0.4231, 0.7589])
		self.current_pose_msg = np.array([0.0, 0.0, 0.0])  #np.array([0.0, -0.4231, 0.7589, 0.0, 0.0, 0.0]) #PoseStamped()
		self.current_target_msg = np.array([0.0, 0.0]) #Float64MultiArray()

		self.origin_position = np.array([0.0, 0.0, 0.0]) #np.array([0.0, -0.4231, 0.7589])
		self.circle_radius = 0.05
		self.point_to_point_time = 4.0 # do point to point motions in this amount of time
		self.circle_loop_times = [4.0, 2.0] # in seconds
		self.hold_time = 1.0

		self.point_to_point_idx_count = int(self.point_to_point_time / self.dt)
		self.circle_idx_counts = [int(self.circle_loop_times[i] / self.dt) for i in range(0,len(self.circle_loop_times))]
		self.hold_idx_count = int(self.hold_time / self.dt)

		# control gains
		self.P_gain = 20.0
		self.D_gain = 1.0
		self.I_gain = 0.5

		self.error_vec = np.zeros_like(self.origin_position)
		self.error_integral_vec = np.zeros_like(self.origin_position)
		self.error_derivative_vec = np.zeros_like(self.origin_position)
		return

	def lerp_list(self, start_point, end_point, alpha):
		lerp_point = [start_point[i] * (alpha - 1) + end_point[i] * (alpha) for i in range(0, len(start_point))]
		return lerp_point

	def lerp_array(self, start_point, end_point, alpha):
		lerp_point = start_point * (alpha - 1) + end_point * (alpha)
		return lerp_point

	def updateCurrentPose(self, data):
		self.current_pose_msg = data.data
		return

	#def updateCurrentPose(self, data):
	#	self.current_pose_msg = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
	#	return

	# it's inefficient to make a new array every call. change when there's more time.
	def getCurrentPosition(self, as_array = False):
		if (as_array):
			abs_pose = np.array([self.current_pose_msg[0], self.current_pose_msg[1], self.current_pose_msg[2]]) - self.initial_pose
			rel_pose = np.array([abs_pose[2], abs_pose[1], -abs_pose[0]])
			return rel_pose #np.array([self.current_pose_msg[0], self.current_pose_msg[1], self.current_pose_msg[2]]) - self.initial_pose
		else:
			return [self.current_pose_msg[0], self.current_pose_msg[1], self.current_pose_msg[2]]

	def updateCurrentTarget(self, data):
		self.current_target_msg = data.data
		return

	def getCurrentTarget(self, as_array = False):
		if (as_array):
			return np.array([self.current_target_msg[0], self.current_pose_msg[1], self.current_target_msg[1]])
		else:
			return [self.current_target_msg[0], self.current_pose_msg[1], self.current_target_msg[1]]

	def updateErrorSignals(self):
		last_error_vec = self.error_vec.copy()
		self.error_vec = self.getCurrentTarget(as_array=True) - self.getCurrentPosition(as_array=True)
		self.error_derivative_vec = self.dt * (self.error_vec - last_error_vec)
		self.error_integral_vec = self.error_integral_vec + self.dt * self.error_vec
		return

	def getControlSignal(self):
		return self.P_gain * self.error_vec + self.D_gain * self.error_derivative_vec + self.I_gain * self.error_integral_vec

	def publish_handler(self, force):
		self.msg = self.top_type()
		self.msg.wrench.force.x = force[2]
		self.msg.wrench.force.y = force[0] # this indexing is intentional and basically converts the frame to be relative to the force sensor and not the base frame
		#self.msg.wrench.force.x = force[0]
		#self.msg.wrench.force.z = force[2]
		self.msg.header.stamp = rospy.Time.now()
		self.pub.publish(self.msg)
		#rospy.loginfo("Published from subject_sim.py!")
		return

	def start_node(self):
        #idx = 0
		rospy.loginfo("SimSubject: Starting main loop...")
		while (not rospy.is_shutdown()):
			#if idx == max_idx:
			#	rospy.loginfo("Pose_Publisher: Finished with commanded trajectory!")
			#	break
			#rospy.loginfo("In subject_sim.py start_node loop!")
			time_start = time.time()
			self.updateErrorSignals()
			control_force = self.getControlSignal()
			#rospy.loginfo("SimSubject: error_vec is " + str(self.error_vec))
			#rospy.loginfo("SimSubject: control_force is " + str(control_force))
			self.publish_handler(control_force)

			#idx += 1
			while (time.time() - time_start) < self.dt:
				pass
		return

if __name__ == "__main__":
	#rospy.loginfo("In subject_sim.py before object def!")
	PubNode = SubjectSimPublisher()
	PubNode.start_node()
