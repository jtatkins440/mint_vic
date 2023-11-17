#!/usr/bin/env python3
# license removed for brevity
import math
import rospy
from std_msgs.msg import String
from std_msgs.msg import MultiArrayDimension
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
import tf
import tf2_ros
import numpy as np
from collections import deque
from motion_intention.msg import HistoryStamped
import time

class HistoryHandler:
    def __init__(self):
        self.nh = rospy.init_node('history_handler', anonymous=True)

        self.state_dim_ = rospy.get_param("/mint/state_dim", 2)
        self.seq_length_ = rospy.get_param("/mint/seq_length", 125)
        self.history_rate = rospy.get_param("/mint/history_rate", 200)
        self.history_dt = 1.0 / (self.history_rate)
        self.rate = rospy.Rate(self.history_rate)

        self.position_current = np.zeros([self.state_dim_, 1])
        self.velocity_current = np.zeros([self.state_dim_, 1])
        self.acceleration_current = np.zeros([self.state_dim_, 1])
        self.history_dim = 3 * self.state_dim_

        self.istate_current = np.zeros([self.history_dim, 1]) # input_state
        self.istate_old = np.zeros([self.history_dim, 1]) # input_state
        self.istate_deque = deque([], maxlen=self.seq_length_)

        self.position_updated = False
        self.velocity_updated = False
        self.acceleration_updated = False

        self.pub = rospy.Publisher("/ee_history", HistoryStamped, queue_size=1)
        self.sub_pose = rospy.Subscriber("/iiwa/ee_pose", PoseStamped, self.callbackPose)
        self.sub_twist = rospy.Subscriber("/iiwa/ee_vel", TwistStamped, self.callbackTwist)
        self.sub_acc = rospy.Subscriber("/iiwa/ee_acc", TwistStamped, self.callbackAcc)

        self.time_start = time.time()

        self.pos_timer = time.time()
        self.vel_timer = time.time()
        self.acc_timer = time.time()
        self.new_value_tol = 0.1

    '''
    def callbackPose(self, msg):
        time_diff = time.time() - self.pos_timer
        position_new = self.position_current.copy()
        if ((self.position_updated == False) and (self.history_dt < time_diff)):
            position_new[0,0] = msg.pose.position.x
            position_new[1,0] = msg.pose.position.y
            diff = np.abs(self.position_current - position_new)
            #print(position_new)
            #print(f"diff: {diff}")
            if (np.all(self.new_value_tol < (diff))): # check if new value is same as from old value, if so don't say it's been updated
                self.position_current = position_new.copy()
                self.position_updated = True
            self.pos_timer = time.time()
        return

    def callbackTwist(self, msg):
        time_diff = time.time() - self.vel_timer
        if ((self.velocity_updated == False) and (self.history_dt < time_diff)):
            self.velocity_current[0,0] = msg.twist.linear.x
            self.velocity_current[1,0] = msg.twist.linear.y
            self.velocity_updated = True
            self.vel_timer = time.time()
        return

    def callbackAcc(self, msg):
        time_diff = time.time() - self.acc_timer
        if ((self.acceleration_updated == False) and (self.history_dt < time_diff)):
            self.acceleration_current[0,0] = msg.twist.linear.x # is a twist message but it's holding acc info.
            self.acceleration_current[1,0] = msg.twist.linear.y
            self.acceleration_updated = True
            self.acc_timer = time.time()
        return
    '''

    def callbackPose(self, msg):
        position_new = self.position_current.copy()
        if (self.position_updated == False):
            position_new[0,0] = msg.pose.position.x
            position_new[1,0] = msg.pose.position.z # technically kuka reaching tasks are in xz-plane
            diff = np.abs(self.position_current - position_new)
            #if (np.all(self.new_value_tol < (diff))): # check if new value is same as from old value, if so don't say it's been updated
            self.position_current = position_new.copy()
            self.position_updated = True
        return

    def callbackTwist(self, msg):
        velocity_new = self.velocity_current.copy()
        if ((self.velocity_updated == False)):
            velocity_new[0,0] = msg.twist.linear.x
            velocity_new[1,0] = msg.twist.linear.z
            diff = np.abs(self.velocity_current - velocity_new)
            #if (np.all(self.new_value_tol < (diff))): # check if new value is same as from old value, if so don't say it's been updated
            self.velocity_current = velocity_new.copy()
            self.velocity_updated = True
        return

    def callbackAcc(self, msg):
        acceleration_new = self.acceleration_current.copy()
        if ((self.acceleration_updated == False)):
            acceleration_new[0,0] = msg.twist.linear.x
            acceleration_new[1,0] = msg.twist.linear.z
            diff = np.abs(self.acceleration_current - acceleration_new)
            #if (np.all(self.new_value_tol < (diff))): # check if new value is same as from old value, if so don't say it's been updated
            self.acceleration_current = acceleration_new.copy()
            self.acceleration_updated = True
        return

    @staticmethod
    def dequeToPubArray(deque):
        row_size = deque[0].shape[0]
        col_size = deque.maxlen
        #print(deque)
        out_array = [] # python publishes nested lists as arrays, each internal list is a row
        
        out_array = np.zeros([row_size, col_size])
        for col in range(0, col_size):
            vec = deque[col]
            #print(vec)
            for row in range(0, row_size):
                out_array[row, col] = vec[row, 0]
        return out_array

    def updateDeque(self):
        delta_time = time.time() - self.time_start
        if self.history_dt < delta_time:
            #print(f"self.history_dt: {self.history_dt}")
            if (self.position_updated and self.velocity_updated and self.acceleration_updated):
                #print(f"delta_time: {delta_time}")
                self.time_start = time.time()
                self.position_updated = False
                self.velocity_updated = False
                self.acceleration_updated = False

                self.istate_current[0:2,0] = self.position_current[0:2,0].copy()
                self.istate_current[2:4,0] = self.velocity_current[0:2,0].copy()
                self.istate_current[4:6,0] = self.acceleration_current[0:2,0].copy()


                #print(f"position_current: {self.position_current}") #is streaming corectly
                #print(f"velocity_current: {self.velocity_current}") #is streaming corectly
                #print(f"acceleration_current: {self.acceleration_current}") #is streaming corectly
                #print(f"istate: {self.istate_current}") #is streaming corectly
                self.istate_deque.append(self.istate_current.copy())
                #print(self.istate_deque) # is not updating correctly
                

                
                #print(self.istate_current)

                if (len(self.istate_deque) >= self.istate_deque.maxlen):
                    #print(f"istate_deque, 1: {self.istate_deque[1]}")
                    #print(f"istate_deque, -1: {self.istate_deque[-1]}")
                    #print(f"istate_deque, 10: {self.istate_deque[50]}")

                    history_array = self.dequeToPubArray(self.istate_deque)

                    #print(f"history_array[0:4, 0:5]: {history_array[0:4, 0:5]}")
                    self.publishHistoryArray(history_array)

    def publishHistoryArray(self, hist_array):
        hist_msg = HistoryStamped()
        hist_msg.header.stamp = rospy.Time.now()
        hist_msg.header.frame_id = "world"
        hist_msg.state = [self.position_current[0,0], self.position_current[1,0], self.velocity_current[0,0], self.velocity_current[1,0]]
        #hist_msg.history = history_array

        hist_msg.history.layout.data_offset = 0 

        # create two dimensions in the dim array
        hist_msg.history.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

        # dim[0] is the vertical dimension of your matrix
        hist_msg.history.layout.dim[0].label = "pos_vel_acc"
        hist_msg.history.layout.dim[0].size = self.history_dim
        hist_msg.history.layout.dim[0].stride = self.history_dim * 125
        # dim[1] is the horizontal dimension of your matrix
        hist_msg.history.layout.dim[1].label = "samples"
        hist_msg.history.layout.dim[1].size = 125
        hist_msg.history.layout.dim[1].stride = 125

        hist_msg.history.data = hist_array.flatten()
        #hist_msg.state =
        self.pub.publish(hist_msg)

    def mainLoop(self):
        while not rospy.is_shutdown():
            self.updateDeque()
            self.rate.sleep()
            #rospy.spin()

if __name__ == '__main__':
    try:
        print("Starting main...")
        hist_node = HistoryHandler()
        hist_node.mainLoop()
    except rospy.ROSInterruptException:
        pass