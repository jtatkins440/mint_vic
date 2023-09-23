#!/usr/bin/env python3
# license removed for brevity
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def talker(ang_vel = 0.0001):
    pub = rospy.Publisher('ee_pose', PoseStamped, queue_size=10)
    rospy.init_node('test_ee_pose_pub', anonymous=True)
    rate = rospy.Rate(100) 
    while not rospy.is_shutdown():
        pose_s = PoseStamped()
        pose_s.header.stamp = rospy.Time.now()
        pose_s.pose.position.x = math.cos(ang_vel * pose_s.header.stamp.nsecs)
        pose_s.pose.position.y = math.sin(ang_vel * pose_s.header.stamp.nsecs)
        pub.publish(pose_s)
        rate.sleep()
        print("looping...")

if __name__ == '__main__':
    try:
        print("Starting main...")
        talker()
    except rospy.ROSInterruptException:
        pass