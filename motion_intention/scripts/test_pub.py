#!/usr/bin/env python3
# license removed for brevity
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf
import tf2_ros

def talker(ang_vel = 0.00001, radius = 0.01):
    pub = rospy.Publisher('ee_pose', PoseStamped, queue_size=2)
    rospy.init_node('test_ee_pose_pub', anonymous=True)
    rate = rospy.Rate(500)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "fixed"
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0
    broadcaster.sendTransform(static_transformStamped)

    br = tf2_ros.TransformBroadcaster() # for the ee pose
    

    while not rospy.is_shutdown():
        pose_s = PoseStamped()
        pose_s.header.stamp = rospy.Time.now()
        used_time = pose_s.header.stamp.secs + (pose_s.header.stamp.nsecs * 1e-6)
        pose_s.pose.position.x = radius * math.cos(ang_vel * used_time)
        pose_s.pose.position.y = radius * math.sin(ang_vel * used_time)
        pose_s.pose.orientation.w = 1.0
        pose_s.header.frame_id = "ee"
        pub.publish(pose_s)

        # make the same thing but as a transform to visualize in RVIZ
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "ee"
        t.transform.translation.x = pose_s.pose.position.x
        t.transform.translation.y = pose_s.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        br.sendTransform(t)

        rate.sleep()
        print("looping...")

if __name__ == '__main__':
    try:
        print("Starting main...")
        talker()
    except rospy.ROSInterruptException:
        pass