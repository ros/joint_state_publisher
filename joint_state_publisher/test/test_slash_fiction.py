#!/usr/bin/env python
import unittest

import rospy

from sensor_msgs.msg import JointState


class ValuesTestCase(unittest.TestCase):
    def test_joint_state_values(self):
        rospy.init_node('test_joint_state_values', anonymous=True)
        self.joint_state = None
        rospy.Subscriber('/joint_states', JointState, self.callback_state)
        while not self.joint_state:
            rospy.sleep(0.1)

        # The Joint State Publisher should read the zeros parameter set in the launch file
        # and as a result set BOTH joints' zero position to be 0.42 instead of 0.3

        # This first test checks basic functionality of the parameter
        self.assertEqual(0.42, self.joint_state.position[0])

        # The second test checks the use case where the joint name has a slash in it
        self.assertEqual(0.42, self.joint_state.position[1])

    def callback_state(self, state):
        self.joint_state = state


if __name__ == '__main__':
    import rostest
    rostest.rosrun('joint_state_publisher', 'test_joint_states', ValuesTestCase)
