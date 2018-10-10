#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy

from geometry_msgs.msg import PointStamped
from vfhp_local_planner.srv import SetGoal, SetGoalResponse


class PointNode(object):

    def __init__(self, goal_service='set_goal', rviz_topic='clicked_point'):

        rospy.init_node('vfhp_rviz_helper', log_level=rospy.DEBUG)

        rospy.wait_for_service(goal_service)
        self.set_goal_srv = rospy.ServiceProxy(goal_service, SetGoal)
        self.clicked_point_sub = rospy.Subscriber(rviz_topic, PointStamped, self.clicked_point_callback)

    def clicked_point_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        try:
            self.set_goal_srv(True, x, y)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed %s" % e)

    def run(self):
        rospy.spin()

def main():
    node = PointNode()
    node.run()


if __name__ == '__main__':
    main()
