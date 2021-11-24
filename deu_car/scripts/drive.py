#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import cv_bridge
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class Drive:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback
        )
        self.cmd_vel_pub = rospy.Publisher(
            "/cmd_vel_mux/input/teleop", Twist, queue_size=1
        )
        self.stop_line_sub = rospy.Subscriber(
            "/stop_line", Bool, self.stop_line_callback
        )
        self.stop_line_pub = rospy.Publisher("/stop_line", Bool, queue_size=1)
        self.twist = Twist()
        self.velocity = 0.0
        self.is_stop_line_detected = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # cv2.imshow("image", image)

        try:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([255, 255, 255])
            mask = cv2.inRange(hsv, lower_white, upper_white)

            h, w = mask.shape

            mask[0 : (h / 10) * 7, 0:w] = 0
            mask[0:h, 0 : w / 4] = 0
            mask[0:h, w - (w / 4) : w] = 0

            # cv2.imshow("mask", mask)

            _, th = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
            _, contours, _ = cv2.findContours(
                th, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )

            if contours:
                area = max(map(lambda contour: cv2.contourArea(contour), contours))
                rospy.loginfo(area)

                if self.is_stop_line_detected:
                    return

                if 10000 < area:
                    rospy.loginfo("=== STOP LINE DETECTED ===")
                    self.stop_line_pub.publish(True)

        except Exception as e:
            print(e)

        finally:
            # cv2.imshow("window", image)
            cv2.waitKey(1)

    def mask_color(self, image):
        converted_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        mask_white = cv2.inRange(
            converted_image,
            np.array([0, 0, 200]),
            np.array([255, 255, 255]),
        )
        mask_yellow = cv2.inRange(
            converted_image,
            np.array([50, 80, 80]),
            np.array([70, 255, 255]),
        )

        mask = cv2.bitwise_or(mask_white, mask_yellow)
        masked_image = cv2.bitwise_and(image, image, mask=mask)

        return masked_image

    def callback(self):
        self.twist.linear.x = self.velocity
        self.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self.twist)

    def set_velocity(self, velocity):
        self.velocity = velocity

    def stop_line_callback(self, msg):
        self.is_stop_line_detected = msg.data
        rospy.loginfo("-- PAUSE DRIVING --")
        self.set_velocity(0.0)
        rospy.sleep(3)
        rospy.loginfo("-- RESUME DRIVING --")
        self.set_velocity(1.0)
        rospy.sleep(3)
        self.is_stop_line_detected = False

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.callback()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("drive_node")
    drive = Drive()
    drive.run()
