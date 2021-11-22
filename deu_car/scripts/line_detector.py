#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

import cv2
import cv_bridge
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb)
        self.cmd_vel_pub = rospy.Publisher(
            "cmd_vel_mux/input/teleop", Twist, queue_size=1
        )
        self.twist = Twist()

    def image_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("image", image)

        color_image = self.mask_color(image)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        blur_image = cv2.GaussianBlur(gray_image, (21, 21), 0)
        edge_image = cv2.Canny(blur_image, threshold1=50, threshold2=80)
        # region_image = self.region_selection(edge_image)

        # cv2.imshow("color", color_image)
        # cv2.imshow("gray", gray_image)
        # cv2.imshow("blur", blur_image)
        cv2.imshow("edge", edge_image)
        # cv2.imshow("region", region_image)

        lines = self.hough_lines(edge_image)
        # print(lines)
        # self.lane_lines(image, lines)
        # self.draw_lines(image, self.lane_lines(image, lines))
        img = self.draw_lines(image, lines)
        cv2.imshow("img", img)

        detected_lines = np.squeeze(lines)

        # 기울기 구하기
        slope_degree = (
            np.arctan2(
                detected_lines[:, 1] - detected_lines[:, 3],
                detected_lines[:, 0] - detected_lines[:, 2],
            )
            * 180
        ) / np.pi

        # 수평 기울기 제한
        detected_lines = detected_lines[np.abs(slope_degree) < 160]
        slope_degree = slope_degree[np.abs(slope_degree) < 160]

        # 수직 기울기 제한
        detected_lines = detected_lines[np.abs(slope_degree) > 95]
        slope_degree = slope_degree[np.abs(slope_degree) > 95]

        # 필터링된 직선 버리기
        l, r = (
            detected_lines[(slope_degree > 0), :],
            detected_lines[(slope_degree < 0), :],
        )

        temp = np.zeros(shape=(image.shape[0], image.shape[1], 3), dtype=np.uint8)
        l, r = l[:, None], r[:, None]

        if l is not None:
            for line in l:
                x1, y1, x2, y2 = line[0]
                cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 2)
        if r is not None:
            for line in r:
                x1, y1, x2, y2 = line[0]
                cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("temp", temp)

        temp2 = cv2.cvtColor(temp, cv2.COLOR_BGR2GRAY)
        moments = cv2.moments(temp2)
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
            err = cx - image.shape[1] / 2
            self.twist.linear.x = 0.8
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("temp2", image)

        cv2.waitKey(3)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("image", image)

        filtered_image = self.mask_color(image)
        cv2.imshow("filtered_image", filtered_image)

        height, width = image.shape[:2]

        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur_image = cv2.GaussianBlur(filtered_image, (5, 5), 0)
        edge_image = cv2.Canny(blur_image, threshold1=50, threshold2=80)

        # cv2.imshow("gray_image", gray_image)
        # cv2.imshow("blur_image", blur_image)
        # cv2.imshow("edge_image", edge_image)

        # vertices = np.array(
        #     [
        #         (0, height / 5 * 4),
        #         (0, height / 3 * 2),
        #         (width / 3, height / 2),
        #         (width / 3 * 2, height / 2),
        #         (width, height / 3 * 2),
        #         (width, height / 5 * 4),
        #     ],
        #     np.int32,
        # )

        # roi_image = region_of_interest(
        #     canny_image,
        #     vertices,
        # )

        detected_lines = cv2.HoughLinesP(
            edge_image,
            rho=1,
            theta=np.pi / 180,
            threshold=30,
            minLineLength=10,
            maxLineGap=20,
        )

        # line_image = np.zeros(shape=(height, width, 3), dtype=np.uint8)
        self.draw_lines(image, detected_lines)
        cv2.imshow("detected_lines", image)

        # detected_lines = [
        #     Line(line[0][0], line[0][1], line[0][2], line[0][3])
        #     for line in detected_lines
        # ]

        # candidate_lines = [
        #     line for line in detected_lines if 0.5 <= np.abs(line.slope) <= 1.5
        # ]

        # lane_lines = compute_lane_from_candidates(candidate_lines, gray_image.shape)

        # line_image = np.zeros(shape=(height, width))
        # for lane in lane_lines:
        #     lane.draw(line_image)

        # cv2.imshow("line_image", line_image)

        detected_lines = np.squeeze(detected_lines)

        # 기울기 구하기
        slope_degree = (
            np.arctan2(
                detected_lines[:, 1] - detected_lines[:, 3],
                detected_lines[:, 0] - detected_lines[:, 2],
            )
            * 180
        ) / np.pi

        # 수평 기울기 제한
        detected_lines = detected_lines[np.abs(slope_degree) < 160]
        slope_degree = slope_degree[np.abs(slope_degree) < 160]

        # 수직 기울기 제한
        detected_lines = detected_lines[np.abs(slope_degree) > 95]
        slope_degree = slope_degree[np.abs(slope_degree) > 95]

        # 필터링된 직선 버리기
        L_lines, R_lines = (
            detected_lines[(slope_degree > 0), :],
            detected_lines[(slope_degree < 0), :],
        )

        temp = np.zeros(shape=(height, width, 3), dtype=np.uint8)
        L_lines, R_lines = L_lines[:, None], R_lines[:, None]

        if L_lines is not None:
            for line in L_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 5)
        if R_lines is not None:
            for line in R_lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(temp, (x1, y1), (x2, y2), (0, 255, 0), 5)

        cv2.imshow("temp", temp)

        # region_top = height // 10 * 4
        # region_bottom = height // 10 * 8

        # filtered_image[0:region_top, 0:width] = 0
        # filtered_image[region_bottom:height, 0:width] = 0

        temp2 = cv2.cvtColor(temp, cv2.COLOR_BGR2GRAY)
        moments = cv2.moments(temp2)
        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            cv2.circle(image, (cx, cy), 10, (0, 0, 255), -1)
            # err = cx - width / 2
            # self.twist.linear.x = 0.8
            # self.twist.angular.z = -float(err) / 100
            # self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("temp2", image)

        cv2.waitKey(3)

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

    def hough_lines(self, image):
        """
        Determine and cut the region of interest in the input image.
            Parameters:
                image: The output of a Canny transform.
        """
        rho = 1  # Distance resolution of the accumulator in pixels.
        theta = np.pi / 180  # Angle resolution of the accumulator in radians.
        threshold = 30  # Only lines that are greater than threshold will be returned.
        minLineLength = 10  # Line segments shorter than that are rejected.
        maxLineGap = (
            100  # Maximum allowed gap between points on the same line to link them
        )
        return cv2.HoughLinesP(
            image,
            rho=rho,
            theta=theta,
            threshold=threshold,
            minLineLength=minLineLength,
            maxLineGap=maxLineGap,
        )

    def region_selection(self, image):
        """
        Determine and cut the region of interest in the input image.
            Parameters:
                image: An np.array compatible with plt.imshow.
        """
        mask = np.zeros_like(image)
        # Defining a 3 channel or 1 channel color to fill the mask with depending on the input image
        if len(image.shape) > 2:
            channel_count = image.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255
        # We could have used fixed numbers as the vertices of the polygon,
        # but they will not be applicable to images with different dimesnions.
        rows, cols = image.shape[:2]
        bottom_left = [cols * 0.1, rows * 0.95]
        top_left = [cols * 0.4, rows * 0.6]
        bottom_right = [cols * 0.9, rows * 0.95]
        top_right = [cols * 0.6, rows * 0.6]
        vertices = np.array(
            [[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32
        )
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(image, mask)
        return masked_image

    def average_slope_intercept(self, lines):
        """
        Find the slope and intercept of the left and right lanes of each image.
            Parameters:
                lines: The output lines from Hough Transform.
        """
        left_lines = []
        left_weights = []
        right_lines = []
        right_weights = []

        for line in lines:
            # x1, y1, x2, y2 = line[0]
            # parameters = np.polyfit((x1, x2), (y1, y2), 1)
            # slope = parameters[0]
            # intercept = parameters[1]
            # if slope < 0:
            #     left_lines.append((slope, intercept))
            #     left_weights.append(len(line))
            # else:
            #     right_lines.append((slope, intercept))
            #     right_weights.append(len(line))

            for x1, y1, x2, y2 in line:
                if x1 == x2:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - (slope * x1)
                length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))
                if slope < 0:
                    left_lines.append((slope, intercept))
                    left_weights.append((length))
                else:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))

        left_lane = (
            np.dot(left_weights, left_lines) / np.sum(left_weights)
            if len(left_weights) > 0
            else None
        )
        right_lane = (
            np.dot(right_weights, right_lines) / np.sum(right_weights)
            if len(right_weights) > 0
            else None
        )
        return left_lane, right_lane

    def pixel_points(self, y1, y2, line):
        """
        Converts the slope and intercept of each line into pixel points.
            Parameters:
                y1: y-value of the line's starting point.
                y2: y-value of the line's end point.
                line: The slope and intercept of the line.
        """
        if line is None:
            return None
        slope, intercept = line
        x1 = int((y1 - intercept) / slope)
        x2 = int((y2 - intercept) / slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))

    def lane_lines(self, image, lines):
        """
        Create full lenght lines from pixel points.
            Parameters:
                image: The input test image.
                lines: The output lines from Hough Transform.
        """
        left_lane, right_lane = self.average_slope_intercept(lines)
        y1 = image.shape[0]
        y2 = y1 * 0.6
        left_line = self.pixel_points(y1, y2, left_lane)
        right_line = self.pixel_points(y1, y2, right_lane)
        return left_line, right_line

    def draw_lines(self, image, lines, color=[255, 0, 0], thickness=2):
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(image, (x1, y1), (x2, y2), color, thickness)
        return cv2.addWeighted(image, 1, image, 1, 0)


if __name__ == "__main__":
    rospy.init_node("line_detector")
    line_detector = LineDetector()
    rospy.spin()
