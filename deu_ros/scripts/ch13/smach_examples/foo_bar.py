#!/usr/bin/env python
import time

import rospy
import smach
import smach_ros


class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("Executing state FOO")
        time.sleep(1)
        if self.counter < 3:
            self.counter += 1
            return "outcome1"
        return "outcome2"


class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["outcome2"])

    def execute(self, userdata):
        rospy.loginfo("Executing state BAR")
        return "outcome2"


def main():
    rospy.init_node("smach_example_state_machine")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["outcome4", "outcome5"])

    with sm:
        smach.StateMachine.add(
            "FOO",
            Foo(),
            transitions={
                "outcome1": "BAR",
                "outcome2": "outcome4",
            },
        )
        smach.StateMachine.add(
            "BAR",
            Bar(),
            transitions={
                "outcome2": "FOO",
            },
        )

    sis = smach_ros.IntrospectionServer("FOO_BAR_SIS", sm, "/SM_ROOT")
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
