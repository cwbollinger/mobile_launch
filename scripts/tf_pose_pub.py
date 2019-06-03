#! /usr/bin/env python

import rospy

import math
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
import turtlesim.srv


class PosePublisher(object):
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=1)

    def map_cb(self, msg):
        self.map = msg

    def run(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                r.sleep()
                continue
    
            msg = PoseStamped()
            msg.header = trans.header
            msg.pose.position = trans.transform.translation
            msg.pose.orientation = trans.transform.rotation
	    self.pose_pub.publish(msg)
            r.sleep()
                

if __name__ == "__main__":
    rospy.init_node('pose_pub')
    node = PosePublisher()
    node.run()

