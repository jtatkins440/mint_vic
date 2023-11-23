#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from ...srv import *
from scipy import interpolate


# Global variables
targetXYold = np.zeros(2)
targetXY = np.zeros(2)
endEffectorXY = np.zeros(2)
targets = np.zeros((6, 2))

def targetXYold_callback(msg):
    global targetXYold
    targetXYold = np.array(msg.data)

def targetXY_callback(msg):
    global targetXY
    targetXY = np.array(msg.data)

def endEffectorXY_callback(msg):
    global endEffectorXY
    endEffectorXY = np.array(msg.data)
'''
def targets_callback(msg):
    global targets
    for i in range(6):
        targets[i] = np.array([msg.data[2*i], msg.data[2*i + 1]])
'''

def update_targets(req):
    global targets
    targets = np.column_stack((req.x, req.y))
    return UpdateTargetsResponse(success=True, message="Targets Updated")


def visualize():
    plt.cla()  # Clear the current figure
    plt.xlim(-0.18, 0.18)
    plt.ylim(-0.18, 0.18)

    print(f"Targets received are: {targets}")

    '''# Draw targets
    for target in targets:
        plt.scatter(target[0], target[1], s=100, c='blue', alpha=0.5)

    # Draw current target
    plt.scatter(targetXY[0], targetXY[1], s=100, c='red', alpha=0.5)

    # Draw end effector
    plt.scatter(endEffectorXY[0], endEffectorXY[1], s=50, c='red')

    # Prepare and evaluate the spline
    if len(targets) > 1:
        tck, u = interpolate.splprep([targets[:,0], targets[:,1]], s=0)
        unew = np.linspace(0, 1, 100)
        out = interpolate.splev(unew, tck)
        plt.plot(out[0], out[1], 'k-', lw=2)

    plt.pause(0.001)
    '''


def main():
    rospy.init_node('visualizer_py')

    rospy.Subscriber('/PreviousTargets', Float64MultiArray, targetXYold_callback)
    rospy.Subscriber('/CurrentTargets', Float64MultiArray, targetXY_callback)
    rospy.Subscriber('/CurrentEndEffectorPose', Float64MultiArray, endEffectorXY_callback)
    # rospy.Subscriber('/TrialTargets', Float64MultiArray, targets_callback)
    # Initialize the service
    service = rospy.Service('update_targets', UpdateTargets, update_targets)

    rangex_ = -0.18
    rangex = 0.18
    rangey_ = -0.18
    rangey = 0.18
    global d_r, u_r
    d_r = 0.015  # Radius of each target point
    u_r = 0.005  # Radius of end-effector position

    rate = rospy.Rate(10)  # 10 Hz
    plt.ion()
    plt.show()

    while not rospy.is_shutdown():
        visualize()
        rate.sleep()

    plt.close()

if __name__ == '__main__':
    main()
