#!/usr/bin/env python3

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
                kp[1, 0] = self.parameters.KPML
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
            if hasattr(self.parameters, "KUB"):
                kubInputted = True
                k_UB = self.parameters.KUB
            if hasattr(self.parameters, "RHO"):
                rhoInputted = True
                rho_rate = self.parameters.RHO

            rospy.loginfo('Trial parameters received. Subject: %d, Trial: %d',
                          self.parameters.Subject_Num, self.parameters.Trial_Num)
            self.parameters = None  # Clear received parameters

            # The purportedly "random" array of pathNums should depend on the number of trials we plan on conducting
            # Current shape is (1, 25) and each path number occurs five times
            pathArray = np.array([0, 2, 4, 3, 1, 3, 4, 0, 1, 0, 2, 3, 4, 2, 1, 0, 4, 1, 3, 2, 3, 1, 4, 2, 0])

            # Definition of target points
            targetx = np.array([
                [0, 6, -5, 8, -1, 0, 5.7, -2.2, 3.1, -9, 0],
                [0, 9.6, 0.2, -7, 3.4, 0, 7.3, 2.1, -4, 8.7, 0],
                [0, -9.6, -0.2, 7, -3.4, 0, 9.2, -5.4, 0.4, 5.4, 0],
                [0, -6.4, -1.1, 4.7, -8.9, 0, -9.9, -3.6, -8.6, 5.1, 0],
                [0, -9.6, -0.2, 7, -3.4, 0, 5.7, -2.2, 3.1, -9, 0],
                [0, -6.4, -1.1, 4.7, -8.9, 0, 6, -5, 8, -1, 0],
                [0, -9.9, -3.6, -8.6, 5.1, 0, 9.6, 0.2, -7, 3.4, 0]
            ])

            targety = np.array([
                [0, 8, -7, -9, 8, 0, 9.8, 8.5, -10, 5.2, 0],
                [0, 8.4, -3.8, -6.9, 9.7, 0, 5.6, -9.6, 8.7, 9.8, 0],
                [0, 8.2, -7, -6.1, 9.8, 0, 6.8, -5.8, 8.4, -4, 0],
                [0, 5.6, -9.6, 8.7, 9.8, 0, 8.2, -7, -6.1, 9.8, 0],
                [0, 8.2, -7, -6.1, 9.8, 0, 9.8, 8.5, -10, 5.2, 0],
                [0, 5.6, -9.6, 8.7, 9.8, 0, 8, -7, -9, 8, 0],
                [0, 8.2, -7, -6.1, 9.8, 0, 8.4, -3.8, -6.9, 9.7, 0]
            ])

            # Array initialization
            meas_torque = np.zeros(7)

            # Variables related to target reaching
            endEffectorXY = np.array([[0], [0]])
            neutralXY = np.array([[0], [0]])
            targetXY = np.array([[0], [0]])
            targetXYold = np.array([[0], [0]])
            temptarget = np.array([[0.1], [0]])
            ycenter = 0.76
            withinErrorBound = False  # Assuming default as False, adjust as needed
            targetReached = False

            # Force Related Variables
            ftx = 0.0  # Force x-direction (filtered)
            fty = 0.0  # Force y-direction (filtered)
            ftx_un = 0.0  # Force x-direction (unfiltered)
            fty_un = 0.0  # Force y-direction (unfiltered)
            zerox = 0.0  # Force x-baseline
            zeroy = 0.0  # Force y-baseline
            ftx_0 = 0.0  # Part of force filtering calc
            fty_0 = 0.0  # Part of force filtering calc
            al = 0.5  # exponential moving average alpha level

            # Force baseline variables/flags
            firstIt = 0  # first iteration flag
            steady = 0  # Flag to collect average first 2000 samples of forces without moving KUKA

            # Variables related to variable dampings
            dt = 0.001
            b_var = np.array([[30], [30]])  # Variable damping
            b_cons = np.array([[10], [10]])
            Bgroups = np.array([[0], [0]])  # Initializing as zero, adjust as needed
            DEFAULT_Damping = 30.0

            x_new_filt = np.array([[0], [0], [0], [0], [0], [0]])
            x_new_filt_old = np.array([[0], [0], [0], [0], [0], [0]])
            xdot_filt = np.array([[0], [0], [0], [0], [0], [0]])
            xdot_filt_old = np.array([[0], [0], [0], [0], [0], [0]])
            xdotdot_filt = np.array([[0], [0], [0], [0], [0], [0]])
            xdotdot_filt_old = np.array([[0], [0], [0], [0], [0], [0]])

            # Variables related to variable stiffness
            intentsum = 0.0
            distAB = 0.0
            d1 = 0.0
            angleproj = 0.0

            # Assuming rho_rate is defined somewhere above this code
            # For example:
            # rho_rate = 1.0
            rho = rho_rate * 0.01 * 20.0 * np.sqrt(2.0)

            import numpy as np

            # Initializing vectors and matrices
            y_reserved = []
            x_reserved = []
            k_var = np.array([[0], [0]])
            Kgroups = np.array([[0], [0]])  # Initializing as zero, adjust as needed
            xstart = np.array([[0], [0]])
            xend = np.array([[0], [0]])
            xcurrent = np.array([[0], [0]])
            bproj = np.array([[0], [0]])
            aproj = np.array([[0], [0]])
            amirror = np.array([[0], [0]])
            projected = np.array([[0], [0]])
            projected_lin = np.array([[0], [0]])
            tempstiff = np.array([[0], [0]])

            # Circle fitting
            y_test = [7., 6., 8., 7., 5., 7.]
            x_test = [1., 2., 5., 7., 9., 3.]
            ysize = 0  # Assuming default as 0, adjust as needed
            circle_a = 0.0
            circle_b = 0.0
            circle_r = 0.0

            # Circle fitting new criteria
            x_Mean = 0.0
            y_Mean = 0.0
            dist_to_center = 0.0
            ux = 0.0
            uy = 0.0

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
