#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import os
import numpy as np
from smach_ros import SimpleActionState
from sensor_msgs.msg import JointState
import time
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import Float64MultiArray
from enum import Enum
from motion_intention.srv import *
from trial_data_logger.srv import *
from visualizer_py.srv import *



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
        # This will publish the calculated joint angles to the Desired JS topic
        self.desired_joint_pub = rospy.Publisher('/iiwa/PositionController/command', Float64MultiArray, queue_size=1)
        # Initialize the subscriber for CurrentJointSpace
        self.current_joint_angles = []
        # Subscribing to Current Joint Space topic
        rospy.Subscriber('/iiwa/joint_states', JointState, self.current_joint_callback)
        # Setting controller behaviour
        self.controller_toggle_srv = rospy.ServiceProxy('/admit/set_admittance_controller_behavior', SetInt)
        # Toggling IK node
        self.ik_toggle_srv = rospy.ServiceProxy('/ik/toggle_publishing', SetBool)

    def current_joint_callback(self, msg):
        self.current_joint_angles = msg.position

    def lerp(self, A, B, t):
        return [a + (b - a)*t for a, b in zip(A, B)]

    def calculate_max_velocity(self, start_angles, end_angles, total_duration):
        maxvel = max([abs(end - start) for start, end in zip(start_angles, end_angles)]) / total_duration
        return maxvel

    def move_to_origin(self):
        if not self.current_joint_angles:
            rospy.logwarn("Current Joint Angles Unavailable: Cannot move to Origin")
            return 'failed'

        # Using a timed interval instead of sleep()
        rospy.loginfo("Current Joint Angles: {}".format(self.current_joint_angles))
        total_duration = 5.0
        max_velocity = self.calculate_max_velocity(self.current_joint_angles, self.origin_joint_angles, total_duration)
        velocity_threshold = 0.25
        while max_velocity > velocity_threshold:
            total_duration += 1
            max_velocity = self.calculate_max_velocity(self.current_joint_angles, self.origin_joint_angles, total_duration)

        freq = 20
        num_points = int(total_duration*freq)
        time_points = np.linspace(0.0, 1.0, num_points)
        dt = total_duration / num_points

        trajectory = [self.lerp(self.current_joint_angles, self.origin_joint_angles, t) for t in time_points]

        start_time = time.time()
        # May have to make changes to make the message compatible with Desired Joint Space Topic
        for idx, joint_angles in enumerate(trajectory):
            joint_state_msg = Float64MultiArray()
            joint_state_msg.data = joint_angles
            while time.time() - start_time < idx * dt:
                pass
            self.desired_joint_pub.publish(joint_state_msg)
            #rospy.loginfo("Moving to Origin:- Current Joint Angle: {}".format(joint_angles))
            # time.sleep(dt)

        rospy.loginfo("Robot Centered")

    def toggle_ik(self, enable):
        try:
            # resp_controller = self.controller_toggle_srv(enable)
            resp_ik = self.ik_toggle_srv(enable)

            if resp_ik.success:
                rospy.loginfo("Node Successfully Toggled: {}".format("ON" if enable else "OFF"))
            else:
                rospy.loginfo("Toggle Failed: {}".format(resp_ik.message))
        except rospy.ServiceException as e:
            rospy.logerr("Service Call Failed: {}".format(e))

    def set_controller_behaviour(self, value):
        try:
            resp_controller = self.controller_toggle_srv(value)
            if resp_controller.success:
                rospy.loginfo("Controller Behaviour set with value: {}".format(value))
            else:
                rospy.loginfo("Controller Behaviour couldn't be set (It's going to through a phase): {}"
                              .format(resp_controller.message))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def execute(self, userdata):
        # Bring the robot to the center
        time.sleep(4) # Apparently gazebo doesn't work instantenously 
        self.toggle_ik(False)
        self.set_controller_behaviour(0)
        self.move_to_origin()

        self.toggle_ik(True)

        return 'centered'


class Init_Trial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiated'], output_keys=['subject_num', 'fitting_method'])
        # Setting controller behaviour
        self.controller_toggle_srv = rospy.ServiceProxy('/admit/set_admittance_controller_behavior', SetInt)
        self.fitting_service = rospy.ServiceProxy('/mint/set_motion_intention_type', SetInt)
        self.init_logger_service = rospy.ServiceProxy('init_logger', InitLogger)
        

    def execute(self, userdata):
        #print("Hello")
        subject_num = int(input("Enter the Subject's Number: "))
        print("Choose a fitting method:")
        print("1. MIntNet")
        print("2. Linear Fitting")
        print("3. Circle Fitting")
        print("4. Zero Stiffness")
        method = int(input("Enter your choice (1/2/3/4): "))

        methodmsg = SetIntRequest()
        methodmsg.data = method - 1
        # Send the chosen method to the MIntNet node via a service
        if method != 4:
            try:
                resp = self.fitting_service(methodmsg)
                if not resp.success:
                    rospy.logerr("Failed to set fitting method.")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

        # Communicate with the TrialDataLogger node to initialize with the subject number
        #subjectmsg = InitLogger()
        #subjectmsg.data = int(subject_num)
        try:
            resp = self.init_logger_service(subject_num)
            if not resp.success:
                rospy.logerr("Failed to initialize TrialDataLogger with subject number.")
                print("Service call failed here")
        except rospy.ServiceException as e:
            rospy.logerr("Service Call Failed: %s", e)
            print("No it failed here")

        userdata.subject_num = subject_num
        userdata.fitting_method = method
        #print("from the other side")
        return 'initiated'


class BaseTrialState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fitted'], input_keys=['trial_type','subject_num', 'fitting_method'],
                             output_keys=['trial_type'])
        self.endEffector = np.array([[0.0], [0.0]])
        self.sub = rospy.Subscriber('/iiwa/j6_pose_custom', Float64MultiArray, self.end_effector_callback)
        self.target_pub = rospy.Publisher('CurrentTargets', Float64MultiArray, queue_size=10)
        self.prev_target_pub = rospy.Publisher('PreviousTargets', Float64MultiArray, queue_size=10)
        self.trial_targets_pub = rospy.Publisher('TrialTargets', Float64MultiArray, queue_size=10)
        self.force_offset = rospy.ServiceProxy('/admit/set_force_offset', Trigger)
        self.start_logger_service = rospy.ServiceProxy('start_logging', StartLogging)
        self.stop_logger_service = rospy.ServiceProxy('stop_logging', Trigger)
        self.controller_toggle_srv = rospy.ServiceProxy('/admit/set_admittance_controller_behavior', SetInt)
        self.ik_toggle_orientation_srv = rospy.ServiceProxy('/ik/toggle_ignore_orientation', SetBool)
        self.ik_set_behavior_srv = rospy.ServiceProxy('/ik/set_ik_behavior', SetInt)
        self.initial_pose = np.array([0.0,-0.4231, 0.7589])
        self.start_guiding = rospy.ServiceProxy('start_guiding', SetInt)
        self.new_trial = rospy.ServiceProxy('new_trial', SetInt)

    def set_ik_behavior(self, val):
        try:
            resp = self.ik_set_behavior_srv(val)
            if not resp.success:
                rospy.logerr("Failed to set IK Behavior")
        except:
            rospy.logerr("Service call to IK set Behvaior Failed !!")

    def force_offset_service(self):
        try:
            resp = self.force_offset()
            if not resp.success:
                rospy.logerr("Falied to trigger Force Offset Service!!")
        except rospy.ServiceException as e:
            rospy.logerr("Service call to Force Offset Failed !! %s", e)

    def ik_toggle_orientation(self, val):
        try:
            resp = self.ik_toggle_orientation_srv(val)
            if not resp.success:
                rospy.logerr("Failed to toggle ik to ignore orientation!!")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
    def send_target_update(self, x, y):
        rospy.wait_for_service('update_targets')
        try:
            update_targets = rospy.ServiceProxy('update_targets', UpdateTargets)
            resp = update_targets(x, y)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # Does the callback need alteration?
    def end_effector_callback(self, msg):
        current_pose_msg = msg.data
        abs_pose = np.array([current_pose_msg[0], current_pose_msg[1], current_pose_msg[2]]) - self.initial_pose
        self.endEffector[0][0] = abs_pose[2]
        self.endEffector[1][0] = abs_pose[0]

    def is_close_enough(self, coord1, coord2):
        radius_enough = 0.025
        distance = np.linalg.norm(coord1 - coord2)
        #print(distance)
        return distance <= radius_enough
    
    def new_trial_service(self, val):
        try:
            resp = self.new_trial(val)
            if not resp.success:
                rospy.logerr("Failed to communicate new trial")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def start_guiding_service(self, val):
        try:
            resp = self.start_guiding(val)
            if not resp.success:
                rospy.logerr("Failed to start guiding service")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

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
        self.ik_toggle_orientation(True)
        # Set IK to Open Loop
        self.ik_set_behavior_srv(1)
        #Trigger Force offset Service 
        self.force_offset_service()    
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

        total_trials = 14
        targetXY = np.array([[0], [0]])
        targetXYold = np.array([[0], [0]])
        origin_target = np.array([[0.0], [0.0]])
        sanity_target = np.array([[0.1], [0.0]])

        for trial in range(total_trials):
            # Start the trial
            trial_num = trial%7

            targets = pathmap[trial_num + 1]

            target_x_gui = targets[:, 0]
            target_y_gui = targets[:, 1]
            #start_gui_service_time = time.time()
            #timeout = 5
            resp_gui = self.send_target_update(target_x_gui, target_y_gui)
            if resp_gui.success:
                print("Trial Number sent to Visualizer")

            
            '''
            while True:
                resp_gui = self.send_target_update(trial)
                if resp_gui.success:
                    print("Trial Targets sent successfully")
                    break
                else:
                    elapse_time = time.time() - start_gui_service_time
                    if elapsed_time >= timeout:
                        print("Couldn't send targets")
                        break
                    time.sleep(0.01)
            '''

            # User centers the robot to origin
            print("targetXY initialized to origin")
            targetXY = origin_target

            # Publishing current targets -- temp -- change it
            target_msg = Float64MultiArray()
            target_msg.data = [targetXY[0][0], targetXY[1][0]]
            self.target_pub.publish(target_msg)

            # Temp start logging call:
            # self.start_logging(userdata.subject_num, trial, userdata.fitting_method, trial_type)

            while not self.is_close_enough(self.endEffector, targetXY):
                # print(f"EE Pose: {self.endEffector} and the Target: {targetXY}")
                #print("User trying to reach origin")
                target_msg = Float64MultiArray()
                target_msg.data = [targetXY[0][0], targetXY[1][0]]
                self.target_pub.publish(target_msg)
                time.sleep(0.001)

            # Publishing previous targets
            prev_target_msg = Float64MultiArray()  # New message for targetXYold
            prev_target_msg.data = [targetXYold[0][0], targetXYold[1][0]]
            self.prev_target_pub.publish(prev_target_msg)

            # Move to sanity check target
            print("TargetXY updated to sanity targets")
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
                #print("User trying to reach sanity target")
                target_msg = Float64MultiArray()
                target_msg.data = [targetXY[0][0], targetXY[1][0]]
                self.target_pub.publish(target_msg)
                time.sleep(0.001)

            targetXYold = sanity_target
            # Start the trial
            # targets = pathmap[trial + 1]

            # Publish targets for GUI
            # trial_targets_msg = Float64MultiArray()
            # for target in targets:
            #     trial_targets_msg.data.extend(target)
            # self.trial_targets_pub.publish(trial_targets_msg)

            # Trigger Trial Data Logger
            self.start_logging(userdata.subject_num, trial, userdata.fitting_method, trial_type)
            # Communicate start of new trials to visualizer
            self.new_trial_service(1)

            for target_count, target in enumerate(targets):
                
                print(f"Target No. {target_count} out of 6")
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
                    #print(f"User trying to reach target no. {target_count}")
                    target_msg = Float64MultiArray()
                    target_msg.data = [targetXY[0][0], targetXY[1][0]]
                    self.target_pub.publish(target_msg)
                    time.sleep(0.001)

                self.start_guiding_service(target_count)

            #End guiding: visualizer refreshes to new trial
            #self.stop_guiding()
            # Wait for 5 seconds before the next trial
            targetXY = np.array([[0], [0]])
            # Trigger Trial Data Logger to close and save the file
            self.stop_logging()
            time.sleep(5)

        return 'fitted'


class Calibration(BaseTrialState):
    def __init__(self):
        super(Calibration, self).__init__()

    def set_controller_behaviour(self, value):
        try:
            resp_controller = self.controller_toggle_srv(value)
            if resp_controller.success:
                rospy.loginfo("Controller Behaviour set with value: {}".format(value))
            else:
                rospy.loginfo("Controller Behaviour couldn't be set (It's going to through a phase): {}"
                              .format(resp_controller.message))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def execute(self, userdata):
        # Toggle MIntNet Node off
        # self.toggle_mintnet(False)
        userdata.trial_type = 'Calibration'
        if userdata.fitting_method == 4:
            self.set_controller_behaviour(1)
        else:
            self.set_controller_behaviour(2)
        super(Calibration, self).execute(userdata)

        # Once trials are completed, toggle MIntNet Node back on
        # self.toggle_mintnet(True)

        return 'fitted'


class Fit_Trial_Block(BaseTrialState):
    def __init__(self):
        super(Fit_Trial_Block, self).__init__()

    def set_controller_behaviour(self, value):
        try:
            resp_controller = self.controller_toggle_srv(value)
            if resp_controller.success:
                rospy.loginfo("Controller Behaviour set with value: {}".format(value))
            else:
                rospy.loginfo("Controller Behaviour couldn't be set (It's going to through a phase): {}"
                              .format(resp_controller.message))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def execute(self, userdata):
        userdata.trial_type = 'Fitting'
        if userdata.fitting_method == 4:
            return 'fitted'
        else:
            self.set_controller_behaviour(3)
        super(Fit_Trial_Block, self).execute(userdata)

        return 'fitted'


def main():
    rospy.init_node('protocol_controller')
    while not rospy.is_shutdown():
        # Create a top-level state machine
        sm_top = smach.StateMachine(outcomes=['success', 'failure'], input_keys=[], output_keys=[])

        sm_top.userdata.fitting_method = 0
        sm_top.userdata.subject_num = 0

        with sm_top:
            smach.StateMachine.add('INITIAL_STATE', Initial_State(),
                                transitions={'initialized': 'ORIGIN_HOLDING', 'failed': 'failure'})

            smach.StateMachine.add('ORIGIN_HOLDING', Origin_Holding(),
                                transitions={'centered': 'TRIAL_SET', 'failed': 'failure'})

            # Trial Set State Machine (Hierarchical)
            sm_trial_set = smach.StateMachine(outcomes=['trial_complete', 'trial_failed'],
                                            input_keys=['subject_num', 'fitting_method'], output_keys=['trial_type', 'subject_num', 'fitting_method'])
            
            with sm_trial_set:
                smach.StateMachine.add('INIT_TRIAL', Init_Trial(), transitions={'initiated': 'CALIBRATION'})
                smach.StateMachine.add('CALIBRATION', Calibration(),
                                    transitions={'fitted': 'FIT_TRIAL_BLOCK'})

                # Fit trial block can be chosen based on the method: 'circle_fitting', 'linear_fitting', 'neural_network'
                smach.StateMachine.add('FIT_TRIAL_BLOCK', Fit_Trial_Block(),
                                    transitions={'fitted': 'trial_complete'})

            smach.StateMachine.add('TRIAL_SET', sm_trial_set,
                                transitions={'trial_complete': 'success', 'trial_failed': 'failure'})

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM-ROOT')
        sis.start()

        # Execute the state machine
        try:
            outcome = sm_top.execute()
        except rospy.ROSInterruptException:
            pass
        finally:
            sis.stop()
            rospy.signal_shutdown("State Machine execution completed or terminated")

        # Wait for ctrl-c to stop the application
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

