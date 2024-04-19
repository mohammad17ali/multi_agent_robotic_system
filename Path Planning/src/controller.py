#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from goal_publisher.msg import PointArray

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node')
        self.pub_goal_switch = rospy.Publisher('/goals_switch', Bool, queue_size=1)
        self.sub_goal = rospy.Subscriber("/goals", PointArray, self.goal_callback)
        self.goal_array = None
        self.goal_index = 0

    def goal_callback(self, msg):
        self.goal_array = msg.goals

    def switch_goal(self):
        if self.goal_array:
            if self.goal_index < len(self.goal_array):
                goal = self.goal_array[self.goal_index]
                rospy.loginfo(f"Switching to goal: {goal}")
                self.pub_goal_switch.publish(True)
                self.goal_index += 1
            else:
                rospy.loginfo("All goals reached!")
                rospy.signal_shutdown("All goals reached!")

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.switch_goal()
            rate.sleep()

if __name__ == '__main__':
    controller = ControllerNode()
    controller.run()
