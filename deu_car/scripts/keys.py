#!/usr/bin/env python
# -*- coding: utf-8 -*-
import select
import sys
import termios
import tty

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

key_mapping = {
    "1": {"x": -6.25, "y": 10.5, "z": 0},
    "2": {"x": -7.4, "y": 10.5, "z": 0},
}

if __name__ == "__main__":
    rospy.init_node("keyboard_driver")
    rospy.wait_for_service("/gazebo/set_model_state")
    try:
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print("Publishing keystrokes. Press Ctrl-C to exit...")

        # TODO: 키보드 입력을 통한 위치 변경은 출발 이전에만 가능함
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                key = sys.stdin.read(1)
                values = key_mapping[key]

                state_msg = ModelState()
                state_msg.model_name = "mobile_base"

                state_msg.pose.position.x = values["x"]
                state_msg.pose.position.y = values["y"]
                state_msg.pose.position.z = values["z"]

                set_state(state_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    except rospy.ServiceException:
        print("Service call failed")
