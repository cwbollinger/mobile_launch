#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from tf import TransformListener


class Follower():
    def __init__(self, path_topic, cmd_topic, april_tag_frame1, april_tag_frame2, num_times_spin, found_switch_topic):
        self.tf_listener = TransformListener()
        self.april_tag_frame1 = april_tag_frame1
        self.april_tag_frame2 = april_tag_frame2
        self.num_times_spin = num_times_spin
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        path_sub = rospy.Subscriber(path_topic, Path, self.path_callback)
        self.spin_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.switch_found_pub = rospy.Publisher(found_switch_topic, Bool, queue_size=1)

    def path_callback(self, msg):

        # execute received path to room
        poses = msg.poses
        for pose in poses:

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = pose.header.frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = pose.pose

            self.move_base_client.send_goal(goal)
            success = self.move_base_client.wait_for_result(rospy.Duration(120))
            state = self.move_base_client.get_state()

            if success and state == GoalStatus.SUCCEEDED:
                print('Made it!')
            else:
                move_base.cancel_goal()
                print('Problem...')

        # spin around to check if APRIL tag is visible
        SWITCH_FOUND = False
        for i in range(self.num_times_spin):

            if (tf_listener.frameExists(self.april_tag_frame1) or tf_listener.frameExists(self.april_tag_frame2)):
                SWITCH_FOUND = True
                break

            if not SWITCH_FOUND:
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.x = 0.0
                cmd.angular.y = 0.0
                cmd.angular.z = 0.1
                self.spin_pub.publish(cmd)

                rospy.sleep(.5)

        # publish if switch was found
        switch_found_pub.publish(SWITCH_FOUND)


if __name__ == '__main__':

    rospy.init_node('path_follower')
    path_topic = rospy.get_param('~path_topic')
    cmd_topic = rospy.get_param('~cmd_topic')
    april_tag_frame1 = rospy.get_param('~april_tag_frame1')
    april_tag_frame2 = rospy.get_param('~april_tag_frame2')
    num_times_spin = rospy.get_param('~num_times_spin')
    found_switch_topic = rospy.get_param('~found_switch_topic')
    follower = Follower(path_topic, cmd_topic, april_tag_frame1, april_tag_frame2, num_times_spin, found_switch_topic)

    rospy.spin()
