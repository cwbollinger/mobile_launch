#! /usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid

class Delatcher(object):
    def __init__(self):
        self.map = None
        rospy.Subscriber('/move_base_node/global_costmap/costmap', OccupancyGrid, self.map_cb)
        self.delatch_pub = rospy.Publisher('/costmap_repeating', OccupancyGrid, queue_size=1)

    def map_cb(self, msg):
        self.map = msg

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.map is not None:
                self.delatch_pub.publish(self.map)
            r.sleep()
            

if __name__ == "__main__":
    rospy.init_node('delatcher')
    node = Delatcher()
    node.run()

