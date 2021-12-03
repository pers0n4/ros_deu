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
        self.bar_pub = rospy.Publisher(
            "camera/rgb/image_raw/p2_bar", Image, queue_size=1
        )
        self.twist = Twist()
        self.velocity = 0.0
        self.is_stop_line_detected = False
        self.is_stop_line_passable = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # cv2.imshow("image", image)

        try:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            color_image = self.mask_color(hsv)
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
            blur_image = cv2.GaussianBlur(gray_image, (21, 21), 0)
            edge_image = cv2.Canny(blur_image, threshold1=50, threshold2=80)

            lines = cv2.HoughLinesP(
                edge_image,
                rho=1,
                theta=np.pi / 180,
                threshold=30,
                minLineLength=10,
                maxLineGap=100,
            )

            detected_lines = np.squeeze(lines)

            slope_degree = (
                np.arctan2(
                    detected_lines[:, 1] - detected_lines[:, 3],
                    detected_lines[:, 0] - detected_lines[:, 2],
                )
                * 180
            ) / np.pi

            detected_lines = detected_lines[np.abs(slope_degree) < 160]
            slope_degree = slope_degree[np.abs(slope_degree) < 160]

            detected_lines = detected_lines[np.abs(slope_degree) > 95]
            slope_degree = slope_degree[np.abs(slope_degree) > 95]

            l, r = (
                detected_lines[(slope_degree > 0), :],
                detected_lines[(slope_degree < 0), :],
            )

            # temp = np.zeros(shape=(image.shape[0], image.shape[1], 3), dtype=np.uint8)
            temp = image.copy()
            l, r = l[:, None], r[:, None]

            if l is not None:
                for line in l:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if r is not None:
                for line in r:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 2)

            self.draw_lines(temp, lines)
            cv2.imshow("window", temp)

            # stop line
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
                # rospy.loginfo(area)

                # 정지선이 감지된 경우
                if area > 10000:
                    self.is_stop_line_detected = True
                    if self.is_stop_line_passable:
                        # self.stop_line_pub.publish(False)
                        rospy.loginfo("RESUME DRIVING")
                        self.set_velocity(1.0)
                    else:
                        rospy.loginfo("STOP LINE DETECTED")
                        # self.stop_line_pub.publish(True)
                        rospy.loginfo("PAUSE DRIVING")
                        self.set_velocity(0.0)
                        rospy.sleep(3.0)
                        rospy.loginfo("STOP LINE PASSABLE")
                        self.is_stop_line_passable = True
                        return
                else:
                    self.is_stop_line_passable = False
            else:
                self.is_stop_line_detected = False
                self.is_stop_line_passable = False

            # blocking bar
            lower_red = np.array([0, 0, 90])
            upper_red = np.array([5, 5, 110])
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # cv2.imshow("mask", mask)

            h, w, _ = image.shape

            mask[0:180, 0:w] = 0
            mask[240:h, 0:w] = 0
            mask[0:h, 0:250] = 0

            M = cv2.moments(mask)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
                self.set_velocity(0.0)
                rospy.loginfo("detecting bar...")
                return
            bar_image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.bar_pub.publish(bar_image_msg)
            self.set_velocity(1.0)

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

    def draw_lines(self, image, lines, color=[255, 0, 0], thickness=2):
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(image, (x1, y1), (x2, y2), color, thickness)
        return cv2.addWeighted(image, 1, image, 1, 0)

    def callback(self):
        self.twist.linear.x = self.velocity
        self.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self.twist)

    def set_velocity(self, velocity):
        self.velocity = velocity

    def stop_line_callback(self, msg):
        self.is_stop_line_detected = msg.data
        if self.is_stop_line_detected:
            rospy.loginfo("PAUSE DRIVING")
            self.set_velocity(0.0)
            rospy.sleep(3.0)
            self.is_stop_line_passable = True
        else:
            rospy.loginfo("RESUME DRIVING")
            self.set_velocity(1.0)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.callback()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("drive_node")
    drive = Drive()
    drive.run()
