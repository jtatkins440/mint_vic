#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np
from smach_ros import SimpleActionState
from my_package.msg import TrialParameters  # Placeholder for parameter template


# Define the individual states
class Initial_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized'],
                             output_keys=['DEFAULT_KP_AP', 'DEFAULT_KN_AP', 'DEFAULT_KP_ML', 'DEFAULT_KN_ML',
                                          'DEFAULT_k_UB', 'DEFAULT_DELTA', 'DEFAULT_R', 'DEFAULT_rho_rate', 'DEFAULT_B',
                                          'DEFAULT_TRIAL_NUMBER'])

    def execute(self, userdata):
        # Initialize relevant variables
        # Constants
        DEFAULT_KP_AP = 25.0
        DEFAULT_KN_AP = 25.0
        DEFAULT_KP_ML = 25.0
        DEFAULT_KN_ML = 25.0
        DEFAULT_k_UB = 300
        DEFAULT_DELTA = 2.94
        DEFAULT_R = 150
        DEFAULT_rho_rate = 0.075
        DEFAULT_B = 10.0;
        DEFAULT_TRIALNUMBER = 0

        # Discuss what other utility this initial state could possibly serve

        return 'initialized'


class Origin_Holding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered'])

    def execute(self, userdata):
        # Bring the robot to the center
        return 'centered'


class Init_Trial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiated', 'aborted'],
                             input_keys=['DEFAULT_KP_AP', 'DEFAULT_KN_AP', 'DEFAULT_KP_ML', 'DEFAULT_KN_ML',
                                         'DEFAULT_k_UB', 'DEFAULT_DELTA', 'DEFAULT_R', 'DEFAULT_rho_rate', 'DEFAULT_B',
                                         'DEFAULT_TRIAL_NUMBER'])

        self.subscriber = rospy.Subscriber("trial_parameters_topic", TrialParameters,
                                           self.param_callback)  # Topic name is a placeholder
        self.parameters = None  # A place to store received parameters

    def param_callback(self, msg):
        """Callback function for parameter subscription."""
        self.parameters = msg

    def execute(self, userdata):
        # Define variables
        # Stiffness and Damping Matrices
        kp = np.array([[DEFAULT_KP_AP], [DEFAULT_KP_ML]])
        kn = np.array([[DEFAULT_KN_AP], [DEFAULT_KN_ML]])
        b_mat = np.array([DEFAULT_B], [DEFAULT_B])

        # Variables
        r = DEFAULT_R
        delta = DEFAULT_DELTA
        k_UB = DEFAULT_k_UB
        rho_rate = DEFAULT_rho_rate
        trialNum = DEFAULT_TRIAL_NUMBER
        subjectNum = 1

        # Input flags
        kpInputted_ML = False
        kpInputted_AP = False
        knInputted_ML = False
        knInputted_AP = False
        bInputted = False
        trialNumberInputted = False
        tuneInputted = False
        deltaInputted = False
        rInputted = False
        rhoInputted = False
        kubInputted = False

        rospy.loginfo('Waiting for trial parameters...')
        while self.parameters is None and not rospy.is_shutdown():
            # Wait for parameters to be received
            rospy.sleep(0.1)  # Sleep to prevent busy waiting

        if self.parameters is not None:
            # Initialize accordingly
            subjectNum = self.parameters.Subject_Num
            trialNum = self.parameters.Trial_Num

            # Optionally handle other parameters
            if hasattr(self.parameters, "KPAP"):
                kpInputted_AP = True
                kp[0, 0] = self.parameters.KPAP
            if hasattr(self.parameters, "KPML"):
                kpInputted_ML = True
                kp[1, 0] = self.parameters.KPML;
            if hasattr(self.parameters, "KNAP"):
                knInputted_AP = True
                kn[0, 0] = self.parameters.KNAP
            if hasattr(self.parameters, "KNML"):
                knInputted_ML = True
                kn[1, 0] = self.parameters.KNML
            if hasattr(self.parameters, "DELTA"):
                deltaInputted = True
                delta = self.parameters.DELTA
            if hasattr(self.parameters, "R"):
                rInputted = True
                r = self.parameters.R
            if hasattr(self.parameters, "DAMPING"):
                bInputted = True
                b_mat[0, 0] = b_mat[1, 0] = self.parameters.DAMPING
            if hasattr((self.parameters, "KUB")):
                kubInputted = True
                k_UB = self.parameters.KUB
            if hasattr(self.parameters, "RHO"):
                rhoInputted = True
                rho_rate = self.parameters.RHO


            rospy.loginfo('Trial parameters received. Subject: %d, Trial: %d',
                          self.parameters.Subject_Num, self.parameters.Trial_Num)
            self.parameters = None  # Clear received parameters
            return 'initiated'
        else:
            rospy.logwarn('Aborted due to ROS shutdown.')
            return 'aborted'


class Calibration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['calibrated', 'recalibrate'])

    def execute(self, userdata):
        # Calibrate stiffness value
        return 'calibrated'  # or 'recalibrate'


class Fit_Trial_Block(smach.State):
    def __init__(self, fitting_method):
        smach.State.__init__(self, outcomes=['fitted'])
        self.fitting_method = fitting_method

    def execute(self, userdata):
        # Conduct fitting based on self.fitting_method
        return 'fitted'


def main():
    rospy.init_node('protocol_controller')

    # Create a top-level state machine
    sm_top = smach.StateMachine(outcomes=['success', 'failure'])

    with sm_top:
        smach.StateMachine.add('INITIAL_STATE', Initial_State(), transitions={'initialized': 'ORIGIN_HOLDING'})

        smach.StateMachine.add('ORIGIN_HOLDING', Origin_Holding(), transitions={'centered': 'TRIAL_SET'})

        # Trial Set State Machine (Hierarchical)
        sm_trial_set = smach.StateMachine(outcomes=['trial_complete', 'trial_failed'])
        with sm_trial_set:
            smach.StateMachine.add('INIT_TRIAL', Init_Trial(), transitions={'initiated': 'CALIBRATION'})
            smach.StateMachine.add('CALIBRATION', Calibration(),
                                   transitions={'calibrated': 'FIT_TRIAL_BLOCK', 'recalibrate': 'INIT_TRIAL'})

            # Fit trial block can be chosen based on the method: 'circle_fitting', 'linear_fitting', 'neural_network'
            smach.StateMachine.add('FIT_TRIAL_BLOCK', Fit_Trial_Block('circle_fitting'),
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
