#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import numpy as np
from smach_ros import SimpleActionState
# Have assumed that current and desired JS topics have JointState msg type - change as required
from sensor_msgs.msg import JointState
import time
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float64MultiArray
from enum import Enum
# Placeholder for fitting method service - change package name and service as required
from your_package_name.srv import SetFittingMethod
# Placeholder for logging services
from trial_data_logger.srv import StartLogging, InitLogger, StopLogging


class FittingMethod(Enum):
    MINTNET = 1
    CIRCLE_FITTING = 2
    LINEAR_FITTING = 3


# Define the individual states
class Initial_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'failed'])

    def execute(self, userdata):
        # Initialize relevant variables
        return 'initialized'


class Origin_Holding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered', 'failed'])
        self.origin_joint_angles = [-1.5708, 1.5708, 0, 1.5708, 0, -1.5708, -0.958709]
        # Placeholder - would need alteration
        self.desired_joint_pub = rospy.Publisher('DesiredJointSpace', JointState, queue_size=1)
        # Initialize the subscriber for CurrentJointSpace
        self.current_joint_angles = []
        # Placeholder - would need alteration
        rospy.Subscriber('CurrentJointSpace', JointState, self.current_joint_callback)
        # Topic name is a placeholder
        self.controller_toggle_srv = rospy.ServiceProxy('toggle_admittance_controller', SetBool)
        self.ik_toggle_srv = rospy.ServiceProxy('toggle_ik', SetBool)

    def current_joint_callback(self, msg):
        self.current_joint_angles = msg

    def lerp(A, B, t):
        return A + t * (B - A)

    def move_to_origin(self):
        if not self.current_joint_angles:
            rospy.logwarn("Current Joint Angles Unavailable: Cannot move to Origin")
            return 'failed'

        # Using a timed interval instead of sleep()
        rospy.loginfo("Current Joint Angles: {}".format(self.current_joint_angles))
        num_points = 100
        time_points = np.linespace(0.0, 1.0, num_points)
        total_duration = 5.0

        dt = total_duration / num_points

        trajectory = [lerp(self.current_joint_angles, self.origin_joint_angles, t) for t in time_points]

        # May have to make changes to make the message compatible with Desired Joint Space Topic
        for joint_angles in trajectory:
            joint_state = JointState()
            joint_state.position = joint_angles
            self.desired_joint_pub.publish(joint_state)
            rospy.loginfo("Moving to Origin:- Current Joint Angle: {}".format(joint_angles))
            time.sleep(dt)

        rospy.loginfo("Robot Centered")

    # Validate if this implementation would work
    def toggle_controller_ik(self, enable):
        try:
            resp_controller = self.controller_toggle_srv(enable)
            resp_ik = self.ik_toggle_srv

            if resp_ik.success and resp_controller.success:
                rospy.loginfo("Nodes Successfully Toggled: {}".format("ON" if enable else "OFF"))
            else:
                rospy.loginfo("Toggle Failed: {}".format(resp.message))
        except rospy.ServiceException as e:
            rospy.logerr("Service Call Failed: {}".format(e))

    def execute(self, userdata):
        # Bring the robot to the center
        self.toggle_controller_ik(False)

        self.move_to_origin()

        self.toggle_controller_ik(True)

        return 'centered'


class Init_Trial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiated'], output_keys=['subject_num', 'fitting_method'])
        # Replace placeholder service type
        self.fitting_service = rospy.ServiceProxy('fitting_method_service', SetFittingMethod)
        self.init_logger_service = rospy.ServiceProxy('init_logger', InitLogger)

    def execute(self, userdata):
        subject_num = input("Enter the Subject's Number: ")
        print("Choose a fitting method:")
        print("1. MIntNet")
        print("2. Circle Fitting")
        print("3. Linear Fitting")
        choice = input("Enter your choice (1/2/3): ")

        if choice == "1":
            method = FittingMethod.MINTNET.value
        elif choice == "2":
            method = FittingMethod.CIRCLE_FITTING.value
        elif choice == "3":
            method = FittingMethod.LINEAR_FITTING.value
        else:
            print("Invalid choice. Defaulting to MIntNet.")
            method = FittingMethod.MINTNET.value

        # Send the chosen method to the MIntNet node via a service
        try:
            resp = self.fitting_service(method)
            if not resp.success:
                rospy.logerr("Failed to set fitting method.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        # Communicate with the TrialDataLogger node to initialize with the subject number
        try:
            resp = self.init_logger_service(subject_num)
            if not resp.success:
                rospy.logerr("Failed to initialize TrialDataLogger with subject number.")
        except rospy.ServiceException as e:
            rospy.logerr("Service Call Failed: %s", e)

        return 'initiated'


class BaseTrialState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fitted'], input_keys=['subject_num', 'fitting_method'],
                             output_keys=['trial_type'])
        self.endEffector = np.array([[0.0], [0.0]])
        # Post stamped msg:- header, pose
        # Following sub/pub declaration need alteration
        self.sub = rospy.Subscriber('CurrentEndEffectorPose', Float64MultiArray, self.end_effector_callback)
        self.target_pub = rospy.Publisher('CurrentTargets', Float64MultiArray, queue_size=10)
        self.prev_target_pub = rospy.Publisher('PreviousTargets', Float64MultiArray, queue_size=10)
        self.trial_targets_pub = rospy.Publisher('TrialTargets', Float64MultiArray, queue_size=10)
        self.start_logger_service = rospy.ServiceProxy('start_logging', StartLogging)
        # StopLogging is a custom srv, and we can use a standard Trigger instead for this
        self.stop_logger_service = rospy.ServiceProxy('stop_logging', StopLogging)

    def end_effector_callback(self, msgs):
        self.endEffector[0][0] = msg.data[0]
        self.endEffector[1][0] = msg.data[1]

    def is_close_enough(self, coord1, coord2):
        radius_enough = 0.013
        distance = np.linalg.norm(coord1 - coord2)
        return distance <= radius_enough

    def start_logging(self, subject_num, trial, method=None, trial_type=None):
        try:
            resp = self.start_logger_service(subject_num, trial, method, trial_type)
            if not resp.success:
                rospy.logerr("Failed to start logging.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def stop_logging(self):
        try:
            resp = self.stop_logger_service()
            if not resp.success:
                rospy.logerr("Failed to stop logging.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def execute(self, userdata):
        # Load target data from csv file
        trial_type = userdata.trial_type
        current_directory = os.path.dirname(os.path.realpath(__file__))
        csv_path = os.path.join(current_directory, '..', 'include', 'targets.csv')
        data = np.loadtxt(csv_path, delimiter=",")

        # Waypoints
        targetx_data = data[:, 0] * 0.01
        targety_data = data[:, 1] * 0.01

        pathmap = {}

        for i in range(0, targetx_data.size, 6):
            path_num = (i // 6) + 1
            pathmap[path_num] = np.column_stack((targetx_data[i:i + 6], targety_data[i:i + 6]))

        total_trials = 7
        targetXY = np.array([[0], [0]])
        targetXYold = np.array([[0], [0]])
        origin_target = np.array([[0.0], [0.0]])
        sanity_target = np.array([[0.1], [0.0]])

        for trial in range(total_trials):
            # User centers the robot to origin
            targetXY = origin_target

            # Publishing current targets
            target_msg = Float64MultiArray()
            target_msg.data = [targetXY[0][0], targetXY[1][0]]
            self.target_pub.publish(target_msg)

            while not self.is_close_enough(self.endEffector, targetXY):
                time.sleep(0.01)

            # Publishing previous targets
            prev_target_msg = Float64MultiArray()  # New message for targetXYold
            prev_target_msg.data = [targetXYold[0][0], targetXYold[1][0]]
            self.prev_target_pub.publish(prev_target_msg)

            # Move to sanity check target
            targetXY = sanity_target
            targetXYold = origin_target

            target_msg = Float64MultiArray()
            target_msg.data = [targetXY[0][0], targetXY[1][0]]
            self.target_pub.publish(target_msg)

            # Publishing previous targets
            prev_target_msg = Float64MultiArray()  # New message for targetXYold
            prev_target_msg.data = [targetXYold[0][0], targetXYold[1][0]]
            self.prev_target_pub.publish(prev_target_msg)

            while not self.is_close_enough(self.endEffector, targetXY):
                time.sleep(0.01)

            targetXYold = sanity_target
            # Start the trial
            targets = pathmap[trial + 1]

            # Publish targets for GUI
            trial_targets_msg = Float64MultiArray()
            for target in targets:
                trial_targets_msg.data.extend(target)
            self.trial_targets_pub.publish(trial_targets_msg)

            # Trigger Trial Data Logger
            self.start_logging(userdata.subject_num, trial, userdata.fitting_method, trial_type)

            for target_count, target in enumerate(targets):
                targetXYold = targetXY
                targetXY = target.reshape(2, 1)

                target_msg = Float64MultiArray()
                target_msg.data = [targetXY[0][0], targetXY[1][0]]
                self.target_pub.publish(target_msg)

                # Publishing previous targets
                prev_target_msg = Float64MultiArray()  # New message for targetXYold
                prev_target_msg.data = [targetXYold[0][0], targetXYold[1][0]]
                self.prev_target_pub.publish(prev_target_msg)

                while not self.is_close_enough(self.endEffector, targetXY):
                    time.sleep(0.01)

            # Wait for 5 seconds before the next trial
            targetXY = np.array([[0], [0]])
            # Trigger Trial Data Logger to close and save the file
            self.stop_logging()
            time.sleep(5)

        return 'fitted'


class Calibration(BaseTrialState):
    def __init__(self):
        super().__init__()
        # Service to toggle MIntNet Node
        # Placeholder, will need to change
        self.mintnet_toggle_srv = rospy.ServiceProxy('toggle_mintnet', SetBool)

    def toggle_mintnet(self, enable):
        try:
            resp = self.mintnet_toggle_srv(enable)
            if resp.success:
                rospy.loginfo("MIntNet Node Successfully Toggled: {}".format("ON" if enable else "OFF"))
            else:
                rospy.loginfo("Toggle Failed: {}".format(resp.message))
        except rospy.ServiceException as e:
            rospy.logerr("Service Call Failed: {}".format(e))

    def execute(self, userdata):
        # Toggle MIntNet Node off
        self.toggle_mintnet(False)
        userdata.trial_type = 'Calibration'
        super().execute(userdata)

        # Once trials are completed, toggle MIntNet Node back on
        self.toggle_mintnet(True)

        return 'calibrated'  # or 'recalibrate'


class Fit_Trial_Block(BaseTrialState):
    def __init__(self):
        super().__init__()

    def execute(self, userdata):
        userdata.trial_type = 'Calibration'
        super().execute(userdata)

        return 'fitted'


def main():
    rospy.init_node('protocol_controller')

    # Create a top-level state machine
    sm_top = smach.StateMachine(outcomes=['success', 'failure'], input_keys=[], output_keys=[])

    with sm_top:
        smach.StateMachine.add('INITIAL_STATE', Initial_State(),
                               transitions={'initialized': 'ORIGIN_HOLDING', 'failed': 'failure'})

        smach.StateMachine.add('ORIGIN_HOLDING', Origin_Holding(),
                               transitions={'centered': 'TRIAL_SET', 'failed': 'failure'})

        # Trial Set State Machine (Hierarchical)
        sm_trial_set = smach.StateMachine(outcomes=['trial_complete', 'trial_failed'],
                                          input_keys=['subject_num', 'fitting_method'], output_keys=['trial_type'])
        with sm_trial_set:
            smach.StateMachine.add('INIT_TRIAL', Init_Trial(), transitions={'initiated': 'CALIBRATION'})
            smach.StateMachine.add('CALIBRATION', Calibration(),
                                   transitions={'calibrated': 'FIT_TRIAL_BLOCK', 'recalibrate': 'INIT_TRIAL'})

            # Fit trial block can be chosen based on the method: 'circle_fitting', 'linear_fitting', 'neural_network'
            smach.StateMachine.add('FIT_TRIAL_BLOCK', Fit_Trial_Block(),
                                   transitions={'fitted': 'trial_complete'})

        smach.StateMachine.add('TRIAL_SET', sm_trial_set,
                               transitions={'trial_complete': 'success', 'trial_failed': 'failure'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM-ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
