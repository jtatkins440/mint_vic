#!/usr/bin/env python3
import rospy
import os
import numpy as np
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from visualizer_py.srv import *
from scipy import interpolate
from geometry_msgs.msg import PoseStamped
from motion_intention.msg import AdmitStateStamped

# Global variables
targetXYold = np.zeros(2)
targetXY = np.zeros(2)
endEffectorXY = np.array([[0.0], [0.0]])
# targets = np.zeros((6, 2))
initial_pose = np.array([0.0,-0.4231, 0.7589])

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

targets = pathmap[1]


def targetXYold_callback(msg):
    global targetXYold
    targetXYold = np.array(msg.data)

def targetXY_callback(msg):
    global targetXY
    targetXY = np.array(msg.data)

def endEffectorXY_callback(msg):
    current_pose_msg = msg.pose.position
    abs_pose = np.array([current_pose_msg.x, current_pose_msg.y, current_pose_msg.z]) - initial_pose
    endEffectorXY[0][0] = abs_pose[2]
    endEffectorXY[1][0] = abs_pose[0]
'''
def targets_callback(msg):
    global targets
    for i in range(6):
        targets[i] = np.array([msg.data[2*i], msg.data[2*i + 1]])
'''

def update_targets(req):
    global targets
    targets = np.column_stack((req.x, req.y))
    is_zero = np.all(targets == np.zeros((6, 2)))
    #if is_zero:
    #    return UpdateTargetsResponse(success=False)
    return UpdateTargetsResponse(success=True)


def visualize():
    plt.cla()  # Clear the current figure
    plt.xlim(-0.18, 0.18)
    plt.ylim(-0.18, 0.18)

    print(f"Targets received are: {targets}")
    print(f"End Effector Pose: {endEffectorXY}")

    # Draw targets
    for target in targets:
        plt.scatter(target[0], target[1], s=200, c='blue', alpha=0.5)

    # Draw current target
    plt.scatter(targetXY[0], targetXY[1], s=200, c='orange', alpha=0.5, edgecolors='black')

    # Draw end effector
    plt.scatter(endEffectorXY[0][0], endEffectorXY[1][0], s=50, c='red')
    
    # Prepare and evaluate the spline
    if len(targets) > 1:
        tck, u = interpolate.splprep([targets[:,0], targets[:,1]], s=0)
        unew = np.linspace(0, 1, 100)
        out = interpolate.splev(unew, tck)
        plt.plot(out[0], out[1], 'k-', lw=2)

    plt.pause(0.0001)
    


def main():
    rospy.init_node('visualizer_py')
    service = rospy.Service('update_targets', UpdateTargets, update_targets)
    rospy.Subscriber('/PreviousTargets', Float64MultiArray, targetXYold_callback)
    rospy.Subscriber('/CurrentTargets', Float64MultiArray, targetXY_callback)
    rospy.Subscriber('/iiwa/admit_state', AdmitStateStamped, endEffectorXY_callback)
    # rospy.Subscriber('/iiwa/j6_pose_custom', Float64MultiArray, endEffectorXY_callback) # --> For simulation
    # rospy.Subscriber('/TrialTargets', Float64MultiArray, targets_callback)
    # Initialize the service

    rangex_ = -0.18
    rangex = 0.18
    rangey_ = -0.18
    rangey = 0.18
    global d_r, u_r
    d_r = 0.015  # Radius of each target point
    u_r = 0.005  # Radius of end-effector position

    rate = rospy.Rate(100)  # 10 Hz
    plt.ion()
    plt.show()

    while not rospy.is_shutdown():
        visualize()
        rate.sleep()

    plt.close()

if __name__ == '__main__':
    main()
