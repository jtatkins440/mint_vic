#!/usr/bin/env python3

import rospy
import os
from your_package_name.srv import InitLogger, StartLogging, StopLogging
from std_msgs.msg import Float64MultiArray, String

class TrialDataLogger:

    def __init__(self):
        # ROS Node Initialization
        rospy.init_node('trial_data_logger')

        # Service Definitions
        rospy.Service('init_logger', InitLogger, self.handle_init_logger)
        rospy.Service('start_logging', StartLogging, self.handle_start_logging)
        rospy.Service('stop_logging', StopLogging, self.handle_stop_logging)

        # Subscribers
        self.subscribers = {
            'CurrentJointSpace': rospy.Subscriber('CurrentJointSpace', Float64MultiArray, self.callback),
            'DesiredJointState': rospy.Subscriber('DesiredJointState', Float64MultiArray, self.callback),
            'DesiredEndEffectorPoseVelAcc': rospy.Subscriber('DesiredEndEffectorPoseVelAcc', Float64MultiArray, self.callback),
            'CurrentEndEffectorPose': rospy.Subscriber('CurrentEndEffectorPose', Float64MultiArray, self.callback),
            'MeasuredWench': rospy.Subscriber('MeasuredWench', Float64MultiArray, self.callback),
            'ReferencePose': rospy.Subscriber('ReferencePose', Float64MultiArray, self.callback),
            'PreviousTargets': rospy.Subscriber('PreviousTargets', Float64MultiArray, self.callback),
            'CurrentTargets': rospy.Subscriber('CurrentTargets', Float64MultiArray, self.callback),
            'TrialTargets': rospy.Subscriber('TrialTargets', Float64MultiArray, self.callback)
        }

        # File handle for logging
        self.file_handle = None

    def handle_init_logger(self, req):
        current_directory = os.path.dirname(os.path.abspath(__file__))
        data_directory = os.path.join(current_directory, 'DATA')
        subject_directory = os.path.join(data_directory, f'Subject{req.subject_num}')
        calibration_directory = os.path.join(subject_directory, 'Calibration')
        fitting_directory = os.path.join(subject_directory, 'Fitting')
        methods = ['MIntNet', 'CircleFitting', 'LinearFitting']

        # Create directories if they don't exist
        for directory in [data_directory, subject_directory, calibration_directory, fitting_directory] + [os.path.join(fitting_directory, method) for method in methods]:
            if not os.path.exists(directory):
                os.makedirs(directory)
                os.chmod(directory, 0o777)  # Set full permissions

        return True

    def handle_start_logging(self, req):
        # Determine the directory based on trial type and method
        current_directory = os.path.dirname(os.path.abspath(__file__))
        if req.trial_type == "Calibration":
            file_path = os.path.join(current_directory, 'DATA', f'Subject{req.subject_num}', 'Calibration', f'trial_{req.trial_num}.txt')
        else:
            methods = ['MIntNet', 'CircleFitting', 'LinearFitting']
            file_path = os.path.join(current_directory, 'DATA', f'Subject{req.subject_num}', 'Fitting', methods[req.method-1], f'trial_{req.trial_num}.txt')

        # Open the file for logging
        self.file_handle = open(file_path, 'w')

        return True

    def handle_stop_logging(self, req):
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None

        return True

    def callback(self, msg):
        if self.file_handle:
            data_str = ', '.join(map(str, msg.data))
            self.file_handle.write(f"{msg._type}: {data_str}\n")

if __name__ == '__main__':
    logger = TrialDataLogger()
    rospy.spin()
