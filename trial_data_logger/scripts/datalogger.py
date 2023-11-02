#!/usr/bin/env python3

import rospy
import os
import h5py
from std_srvs.srv import Trigger
from trial_data_logger.srv import InitLogger, StartLogging
from std_msgs.msg import Float64MultiArray


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
            'CurrentJointSpace': rospy.Subscriber('CurrentJointSpace', Float64MultiArray, self.callback),
            # Desired Joint State
            'DesiredJointState': rospy.Subscriber('DesiredJointState', Float64MultiArray, self.callback),
            # Desired EndEffector Pose, Velocity and Acceleration
            'DesiredEndEffectorPoseVelAcc': rospy.Subscriber('DesiredEndEffectorPoseVelAcc', Float64MultiArray,
                                                             self.callback),
            # Current End Effector Pose
            'CurrentEndEffectorPose': rospy.Subscriber('CurrentEndEffectorPose', Float64MultiArray, self.callback),
            # Measured Wench
            'MeasuredWench': rospy.Subscriber('MeasuredWench', Float64MultiArray, self.callback),
            # Reference Pose
            'ReferencePose': rospy.Subscriber('ReferencePose', Float64MultiArray, self.callback),
            # Previous Targets
            'PreviousTargets': rospy.Subscriber('PreviousTargets', Float64MultiArray, self.callback),
            # Current Targets
            'CurrentTargets': rospy.Subscriber('CurrentTargets', Float64MultiArray, self.callback),
            # Trial Targets
            'TrialTargets': rospy.Subscriber('TrialTargets', Float64MultiArray, self.callback)
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
            data_set = self.data_group.create_dataset(msg._type, data=list(msg.data))


if __name__ == '__main__':
    logger = TrialDataLogger()
    rospy.spin()
