#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


class BarDetector:
    def __init__(self):
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback)
        self.range_ahead = 1

    def scan_callback(self, msg):
        self.range_ahead = min(msg.ranges)
        print(msg.ranges)
        print(self.range_ahead)


if __name__ == "__main__":
    rospy.init_node("bar_detector")
    bar_detector = BarDetector()
    rospy.spin()
