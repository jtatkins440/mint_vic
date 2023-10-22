#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import numpy as np
from smach_ros import SimpleActionState
from sensor_msgs.msg import JointState
import time
from std_srvs.srv import SetBool
from std_msgs import Float64MultiArray

# Define the individual states
class Initial_State(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized'])

    def execute(self, userdata):
        # Initialize relevant variables
        return 'initialized'


class Origin_Holding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered', 'failed'])
        self.origin_joint_angles = [-1.5708, 1.5708, 0, 1.5708, 0, -1.5708, -0.958709]
        self.desired_joint_pub = rospy.Publisher('DesiredJointSpace', JointState, queue_size=10)
        # Initialize the subscriber for CurrentJointSpace
        self.current_joint_angles = []
        rospy.Subscriber('CurrentJointSpace', JointState, self.current_joint_callback)
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

        rospy.loginfo("Current Joint Angles: {}".format(self.current_joint_angles))
        num_points = 100
        time_points = np.linespace(0, 1, num_points)
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
        smach.State.__init__(self, outcomes=['initiated'])

    def execute(self, userdata):
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
        self.endEffector = np.array([[0.0], [0.0]])
        self.sub = rospy.Subscriber('CurrentEndEffectorPose', Float64MultiArray, self.end_effector_callback)
        self.target_pub = rospy.Publisher('CurrentTargets', Float64MultiArray, queue_size=10)

    def end_effector_callback(self, msgs):
        self.endEffector[0][0] = msg.data[0]
        self.endEffector[1][0] = msg.data[1]

    def close_enough(self, coord1, coord2):
        radius_enough = 0.013
        distance = np.linalg.norm(coord1 - coord2)
        return distance <= radius_enough

    def execute(self, userdata):
        # Load target data from csv file
        csv_path = 'protocol_controller/include/targets.csv'
        data = np.loadtxt(csv_path, delimiter=",")

        # Waypoints
        targetx_data = data[:, 0] * 0.01
        targety_data = data[:, 1] * 0.01

        pathmap = {}

        for i in range(0, targetx_data.size, 6):
            path_num = (i // 6) + 1
            pathmap[path_num] = np.column_stack((targetx_data[i:i + 6], targety_data[i:i + 6]))

        initial_path = 1
        total_trials = 7
        targetXY = np.array([[0], [0]])
        targetXYold = np.array([[0], [0]])
        temptarget = np.array([[0.1], [0]])
        origin_target = np.array([[0.0], [0.0]])
        sanity_target = np.array([[0.01], [0.0]])

        for trial in range(total_trials):
            # User centers the robot to origin
            targetXY = origin_target

            target_msg = Float64MultiArray()
            target_msg.data = [targetXY[0][0], targetXY[1][0]]
            self.target_pub.publish(target_msg)

            while not is_close_enough(self.endEffector, targetXY):
                time.sleep(0.5)

            # Move to sanity check target
            targetXY = sanity_target

            target_msg = Float64MultiArray()
            target_msg.data = [targetXY[0][0], targetXY[1][0]]
            self.target_pub.publish(target_msg)

            while not is_close_enough(self.endEffector, targetXY):
                time.sleep(0.5)

            # Start the trial
            targets = pathmap[trial + 1]
            for target in targets:
                targetXY = target.reshape(2, 1)

                target_msg = Float64MultiArray()
                target_msg.data = [targetXY[0][0], targetXY[1][0]]
                self.target_pub.publish(target_msg)

                while not is_close_enough(self.endEffector, targetXY):
                    time.sleep(0.5)

            # Wait for 5 seconds before the next trial
            targetXY = np.array([[0], [0]])
            time.sleep(5)

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
