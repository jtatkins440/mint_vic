#!/usr/bin/env python3

import rospy
import os
import h5py
from std_srvs.srv import Trigger
from trial_data_logger.srv import InitLogger, StartLogging
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped


class TrialDataLogger:

    def __init__(self):
        # ROS Node Initialization
        rospy.init_node('trial_data_logger')

        # Service Definitions
        rospy.Service('init_logger', InitLogger, self.handle_init_logger)
        rospy.Service('start_logging', StartLogging, self.handle_start_logging)
        rospy.Service('stop_logging', Trigger, self.handle_stop_logging)  # Using standard Trigger service

        # Subscribers
        self.subscribers = {
            # Current Joint Space
            '/iiwa/joint_states': rospy.Subscriber('/iiwa/joint_states', JointState, self.callback),
            # Desired Joint State
            '/iiwa/PositionController/command': rospy.Subscriber('/iiwa/PositionController/command', Float64MultiArray,
                                                                 self.callback),
            # Desired EndEffector Pose
            '/iiwa/ee_pose': rospy.Subscriber('/iiwa/ee_pose', PoseStamped, self.callback),
            # Current End Effector Pose
            '/iiwa/ee_pose_custom': rospy.Subscriber('/iiwa/ee_pose_custom', Float64MultiArray, self.callback),
            # End Effector Velocity
            '/iiwa/ee_vel': rospy.Subscriber('/iiwa/ee_vel', TwistStamped, self.callback),
            # End Effector Acceleration
            '/iiwa/ee_acc': rospy.Subscriber('/iiwa/ee_acc', TwistStamped, self.callback),
            # Measured Wench
            '/sensor_values': rospy.Subscriber('/sensor_values', WrenchStamped, self.callback),
            # Reference Pose
            '/ee_pose_eq': rospy.Subscriber('ee_pose_eq', PoseStamped, self.callback),
            # Previous Targets
            '/PreviousTargets': rospy.Subscriber('/PreviousTargets', Float64MultiArray, self.callback),
            # Current Targets
            '/CurrentTargets': rospy.Subscriber('/CurrentTargets', Float64MultiArray, self.callback),
            # Trial Targets
            '/TrialTargets': rospy.Subscriber('/TrialTargets', Float64MultiArray, self.callback)
        }

        # File handle for logging
        self.file_handle = None
        self.data_group = None

    def handle_init_logger(self, req):
        current_directory = os.path.dirname(os.path.abspath(__file__))
        data_directory = os.path.join(current_directory, 'DATA')
        subject_directory = os.path.join(data_directory, f'Subject{req.subject_num}')
        calibration_directory = os.path.join(subject_directory, 'Calibration')
        fitting_directory = os.path.join(subject_directory, 'Fitting')
        methods = ['MIntNet', 'CircleFitting', 'LinearFitting']

        # Create directories if they don't exist
        for directory in [data_directory, subject_directory, calibration_directory, fitting_directory] + [
            os.path.join(fitting_directory, method) for method in methods]:
            if not os.path.exists(directory):
                os.makedirs(directory)
                os.chmod(directory, 0o777)  # Set full permissions

        return True

    def handle_start_logging(self, req):
        # Determine the directory based on trial type and method
        current_directory = os.path.dirname(os.path.abspath(__file__))
        if req.trial_type == "Calibration":
            file_path = os.path.join(current_directory, 'DATA', f'Subject{req.subject_num}', 'Calibration',
                                     f'data{req.trial_num}.h5')
        else:
            methods = ['MIntNet', 'CircleFitting', 'LinearFitting']
            file_path = os.path.join(current_directory, 'DATA', f'Subject{req.subject_num}', 'Fitting',
                                     methods[req.method - 1], f'data{req.trial_num}.h5')

        # Open the HDF5 file for logging
        self.file_handle = h5py.File(file_path, 'w')
        self.data_group = self.file_handle.create_group("TrialData")

        return True

    def handle_stop_logging(self, req):
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
            self.data_group = None

        return True

    def callback(self, msg):
        if self.data_group:
            # Convert the data to a list and store in the HDF5 file
            if isinstance(msg, Float64MultiArray):
                data_set = self.data_group.create_dataset(msg._type, data=list(msg.data))
            elif isinstance(msg, PoseStamped):
                data_set = self.data_group.create_dataset(msg._type, data=[msg.pose.position.x, msg.pose.position.y,
                                                                           msg.pose.position.z, msg.pose.orientation.x,
                                                                           msg.pose.orientation.y,
                                                                           msg.pose.orientation.z,
                                                                           msg.pose.orientation.w])
            elif isinstance(msg, TwistStamped):
                data_set = self.data_group.create_dataset(msg._type, data=[msg.twist.linear.x, msg.twist.linear.y,
                                                                           msg.twist.linear.z, msg.twist.angular.x,
                                                                           msg.twist.angular.y, msg.twist.angular.z])
            elif isinstance(msg, WrenchStamped):
                data_set = self.data_group.create_dataset(msg._type, data=[msg.wrench.force.x, msg.wrench.force.y,
                                                                           msg.wrench.force.z, msg.wrench.torque.x,
                                                                           msg.wrench.torque.y, msg.wrench.torque.z])
            elif isinstance(msg, JointState):
                data_set = self.data_group.create_dataset(msg._type, data=list(msg.position))
            else:
                rospy.logwarn(f"Unsupported message type: {type(msg)}")


if __name__ == '__main__':
    logger = TrialDataLogger()
    rospy.spin()
