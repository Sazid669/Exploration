#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class InitialPoseToTarget:
    def __init__(self):
        rospy.init_node("initialpose_to_target")
        self.pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=10)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.callback)
        rospy.loginfo("Ready: Use '2D Pose Estimate' in RViz to send a /target_pose")
        rospy.spin()

    def callback(self, msg):
        target = PoseStamped()
        target.header = msg.header
        target.pose = msg.pose.pose
        self.pub.publish(target)
        rospy.loginfo(f"Published target pose from initialpose: x={target.pose.position.x:.2f}, y={target.pose.position.y:.2f}")

if __name__ == '__main__':
    try:
        InitialPoseToTarget()
    except rospy.ROSInterruptException:
        pass
