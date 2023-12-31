#!/usr/bin/env python3

import rospy
import os
import h5py
import time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from trial_data_logger.srv import *
from std_msgs.msg import Float64MultiArray
from motion_intention.srv import *
from geometry_msgs.msg import PoseStamped, WrenchStamped, TwistStamped


class TrialDataLogger:

    def __init__(self):
        # File handle for logging
        self.file_handle = None
        self.data_group = None

        self.start_time = None

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
            '/iiwa/ee_pose_custom': rospy.Subscriber('/iiwa/j6_pose_custom', Float64MultiArray, self.callback),
            # End Effector Velocity
            '/iiwa/ee_vel': rospy.Subscriber('/iiwa/ee_vel', TwistStamped, self.callback),
            # End Effector Acceleration
            '/iiwa/ee_acc': rospy.Subscriber('/iiwa/ee_acc', TwistStamped, self.callback),
            # Measured Wench
            '/sensor_values': rospy.Subscriber('/sensor_values', WrenchStamped, self.callback),
            # Reference Pose
            '/ee_pose_eq': rospy.Subscriber('/ee_pose_eq', PoseStamped, self.callback),
            # Previous Targets
            '/PreviousTargets': rospy.Subscriber('/PreviousTargets', Float64MultiArray, self.callback),
            # Current Targets
            '/CurrentTargets': rospy.Subscriber('/CurrentTargets', Float64MultiArray, self.callback),
            # Trial Targets
            '/TrialTargets': rospy.Subscriber('/TrialTargets', Float64MultiArray, self.callback),
            # Current Stiffness
            '/current_stiffness': rospy.Subscriber('/current_stiffness', Float64MultiArray, self.callback)
        }



    def handle_init_logger(self, req):
        res = InitLoggerResponse()
        current_directory = os.path.dirname(os.path.abspath(__file__))
        data_directory = os.path.join(current_directory, 'DATA')
        subject_directory = os.path.join(data_directory, f'Subject{req.data}')
        calibration_directory = os.path.join(subject_directory, 'Calibration')
        fitting_directory = os.path.join(subject_directory, 'Fitting')
        methods = ['MIntNet', 'LinearFitting', 'CircleFitting']

        # Create directories if they don't exist
        for directory in [data_directory, subject_directory, calibration_directory, fitting_directory] + [
            os.path.join(fitting_directory, method) for method in methods]:
            if not os.path.exists(directory):
                os.makedirs(directory)
                os.chmod(directory, 0o777)  # Set full permissions

        # Create directories if they don't exist
        for directory in [data_directory, subject_directory, calibration_directory, fitting_directory] + [
            os.path.join(calibration_directory, method) for method in methods]:
            if not os.path.exists(directory):
                os.makedirs(directory)
                os.chmod(directory, 0o777)  # Set full permissions

        res.success = True
        res.message = "Subject Directory created"

        return res

    def handle_start_logging(self, req):
        res = StartLoggingResponse()
        # Determine the directory based on trial type and method
        try:
            current_directory = os.path.dirname(os.path.abspath(__file__))
            methods = ['MIntNet', 'LinearFitting', 'CircleFitting']
            if req.trial_type == "Calibration":
                file_path = os.path.join(current_directory, 'DATA', f'Subject{req.subject_num}', 'Calibration',
                                         methods[req.method-1],
                                         f'data{req.trial_num}.h5')
            else:
                file_path = os.path.join(current_directory, 'DATA', f'Subject{req.subject_num}', 'Fitting',
                                         methods[req.method - 1], f'data{req.trial_num}.h5')
    
            # Open the HDF5 file for logging
            self.file_handle = h5py.File(file_path, 'w')
            self.data_group = self.file_handle.create_group("TrialData")
            
            res.success = True
            res.message = "Started Logging Successfully"
        except Exception as e:
            res.success = False
            res.message = f"Failed to start logging{e}"
            rospy.logerr(f"handle_start_logging: {e}")
        finally:
            self.start_time = time.time() if res.success else None
            
        return res

    def handle_stop_logging(self, req):
        res = TriggerResponse()
        try:
            self.file_handle.close()
            self.file_handle = None
            self.data_group = None
            self.start_time = None
            res.message = "Stopped Logging Successfully"
            res.success = True

        except Exception as e:
            res.success = False
            res.message = "Could close current file"

        return res

    def callback(self, msg):
        if self.data_group is None:
            rospy.logawarn("Data Group is not initialized. Skipping message processing")
            return
        
        if self.start_time is not None:
            elapsed_time = time.time() - self.start_time
            if self.data_group:
                # Use the topic name to create a unique dataset name
                topic_name = msg._connection_header['topic']
                dataset_name = topic_name.replace('/', '_').lstrip(
                    '_')  # Clean up the topic name to be used as a dataset name

                # Prepare the data based on the message type
                if isinstance(msg, Float64MultiArray):
                    data_to_append = [elapsed_time]+list(msg.data)
                elif isinstance(msg, PoseStamped):
                    data_to_append = [elapsed_time]+[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                                    msg.pose.orientation.x, msg.pose.orientation.y,
                                                    msg.pose.orientation.z, msg.pose.orientation.w]
                elif isinstance(msg, TwistStamped):
                    data_to_append = [elapsed_time]+[msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                                                    msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]
                elif isinstance(msg, WrenchStamped):
                    data_to_append = [elapsed_time]+[msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                                    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
                elif isinstance(msg, JointState):
                    data_to_append = [elapsed_time]+list(msg.position)
                else:
                    rospy.logwarn(f"Unsupported message type: {type(msg)}")
                    return

                try:
                    # Check if the dataset exists, if so, append, otherwise create
                    if dataset_name in self.data_group:
                    # Append to the dataset
                        dataset = self.data_group[dataset_name]
                        dataset.resize((dataset.shape[0] + 1, len(data_to_append)))
                        dataset[-1] = data_to_append
                    else:
                        # Create a new dataset
                        try:
                            self.data_group.create_dataset(dataset_name, data=[data_to_append],
                                           maxshape=(None, len(data_to_append)), chunks=True)
                        except ValueError as ve:
                            rospy.logerr(f"Failed to create dataset {dataset_name}: {ve}")
                except KeyError as e:
                    rospy.logerr("Failed to access or create dataset: {}. Error: {}".format(dataset_name, e))
        else:
            rospy.loginfo("Logging hasn't started yet")


if __name__ == '__main__':
    logger = TrialDataLogger()
    rospy.spin()
