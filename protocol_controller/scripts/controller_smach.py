#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState


# Define the individual states
class Initial_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized'])

    def execute(self, userdata):
        # Initialize relevant variables
        return 'initialized'


class Origin_Holding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered'])

    def execute(self, userdata):
        # Bring the robot to the center
        return 'centered'


class Init_Trial(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initiated'])

    def execute(self, userdata):
        # Define variables
        return 'initiated'


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
