#!/usr/bin/env python
import roslib
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def poseCb(msg):
    timestamp = str(msg.header.stamp.secs) + "." + str(msg.header.stamp.nsecs)
    print timestamp,msg.pose.pose.position.x, \
    msg.pose.pose.position.y,msg.pose.pose.position.z, \
    msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, \
    msg.pose.pose.orientation.z,msg.pose.pose.orientation.w

if __name__ == "__main__":
    rospy.init_node('poseToOdom', anonymous=True) #make node
    rospy.Subscriber('/rgbd/pose',PoseWithCovarianceStamped,poseCb)
    rospy.spin()
