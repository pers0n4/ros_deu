#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


class Drive:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher(
            "/cmd_vel_mux/input/teleop", Twist, queue_size=1
        )
        self.twist = Twist()

    def callback(self):
        self.twist.linear.x = 1.0
        self.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.callback()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("drive_node")
    drive = Drive()
    drive.run()
