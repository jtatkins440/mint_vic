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


class SyncDataLogger:

    def __init__(self):
        self.file_handle = None
        self.data_group = None
        self.start_time = None
        self.hz = 200.0
        self.cycle_time = 1.0 / (1.0 * self.hz)

        rospy.Service('init_logger', InitLogger, self.handle_init_logger)
        rospy.Service('start_logging', StartLogging, self.handle_start_logging)
        rospy.Service('stop_logging', Trigger, self.handle_stop_logging)

        self.curr_joint_states = None  # Current Joint Space
        self.desired_joint_states = None  # Desired Joint State
        self.desired_ee_pose = None  # Desired EndEffector Pose
        self.curr_ee_pose = None  # Current End Effector Pose
        self.ee_vel = None  # End Effector Velocity
        self.ee_acc = None  # End Effector Acceleration
        self.int_force = None  # Measured Wench
        self.ref_pose = None  # Reference Pose
        self.prev_targets = None  # Previous Targets
        self.curr_targets = None  # Current Targets
        self.stiffness = None  # Current Stiffness

        self.subscribers = {
            '/iiwa/joint_states': rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_state_callback),

            '/iiwa/PositionController/command': rospy.Subscriber('/iiwa/PositionController/command', Float64MultiArray,
                                                                 self.desired_state_callback),

            '/iiwa/ee_pose': rospy.Subscriber('/iiwa/ee_pose', PoseStamped, self.desired_eepose_callback),

            '/iiwa/ee_pose_custom': rospy.Subscriber('/iiwa/j6_pose_custom', Float64MultiArray, self.current_eepose_callback),

            '/iiwa/ee_vel': rospy.Subscriber('/iiwa/ee_vel', TwistStamped, self.ee_vel_callback),

            '/iiwa/ee_acc': rospy.Subscriber('/iiwa/ee_acc', TwistStamped, self.ee_acc_callback),

            '/sensor_values': rospy.Subscriber('/sensor_values', WrenchStamped, self.int_force_callback),

            '/ee_pose_eq': rospy.Subscriber('/ee_pose_eq', PoseStamped, self.ee_pose_eq_callback),

            '/PreviousTargets': rospy.Subscriber('/PreviousTargets', Float64MultiArray, self.prev_targets_callback),

            '/CurrentTargets': rospy.Subscriber('/CurrentTargets', Float64MultiArray, self.current_targets_callback),

            '/iiwa/current_stiffness': rospy.Subscriber('/iiwa/current_stiffness', Float64MultiArray, self.current_stiffness_callback)
        }

    # Callbacks for each topic
    def joint_state_callback(self, msg):
        self.curr_joint_states = list(msg.position)

    def desired_state_callback(self, msg):
        self.desired_joint_states = list(msg.data)

    def desired_eepose_callback(self, msg):
        self.desired_ee_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z, msg.pose.orientation.w]

    def current_eepose_callback(self, msg):
        self.curr_ee_pose = list(msg.data)

    def ee_vel_callback(self, msg):
        self.ee_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                       msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]

    def ee_acc_callback(self, msg):
        self.ee_acc = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                       msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]

    def int_force_callback(self, msg):
        self.int_force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                          msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]

    def ee_pose_eq_callback(self, msg):
        self.ref_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                         msg.pose.orientation.x, msg.pose.orientation.y,
                         msg.pose.orientation.z, msg.pose.orientation.w]

    def prev_targets_callback(self, msg):
        self.prev_targets = list(msg.data)

    def current_targets_callback(self, msg):
        self.curr_targets = list(msg.data)

    def current_stiffness_callback(self, msg):
        self.stiffness = list(msg.data)

    # Services Handlers - init, start, stop
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
            res.message = f"Could close current file: {e}"

        return res
    
    def wait_for_time(self, start_time_point):
        end_before_rest = time.time()
        elapsed_time = end_before_rest - start_time_point

        while elapsed_time < self.cycle_time:
            elapsed_time = time.time() - start_time_point

    def sync_log(self):
        # if self.data_group is None:
            # rospy.logwarn("Data Group is not initialized. Skipping message processing")

        if self.start_time is not None:
            elapsed_time = time.time() - self.start_time
            if self.data_group:
                for index, topic_name in enumerate(self.subscribers.keys()):
                    dataset_name = topic_name.replace('/', '_').lstrip('_')
                    if index == 0:
                        data_to_append = [elapsed_time] + self.curr_joint_states
                    elif index == 1:
                        data_to_append = [elapsed_time] + self.desired_joint_states
                    elif index == 2:
                        data_to_append = [elapsed_time] + self.desired_ee_pose
                    elif index == 3:
                        data_to_append = [elapsed_time] + self.curr_ee_pose
                    elif index == 4:
                        data_to_append = [elapsed_time] + self.ee_vel
                    elif index == 5:
                        data_to_append = [elapsed_time] + self.ee_acc
                    elif index == 6:
                        data_to_append = [elapsed_time] + self.int_force
                    elif index == 7:
                        data_to_append = [elapsed_time] + self.ref_pose
                    elif index == 8:
                        data_to_append = [elapsed_time] + self.prev_targets
                    elif index == 9:
                        data_to_append = [elapsed_time] + self.curr_targets
                    elif index == 10:
                        data_to_append = [elapsed_time] + self.stiffness

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
                rospy.loginfo("Logging has not been initiated")

    def main(self):
        rospy.init_node('trial_data_logger')
        #rate = rospy.rate(200) # 200Hz
        time_start = time.time()
        self.wait_for_time(time_start)
        self.dt = time.time() - time_start
        while not rospy.is_shutdown():
            self.sync_log()
            #time.sleep(0.005)
            self.wait_for_time(time_start)
            self.dt = time.time() - time_start
            time_start = time.time()


if __name__ == '__main__':
    logger = SyncDataLogger()
    logger.main()


