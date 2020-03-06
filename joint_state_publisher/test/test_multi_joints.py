# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pytest
import rclpy
import sensor_msgs.msg


@pytest.mark.rostest
def generate_test_description():
    test_urdfs_dir = os.path.dirname(__file__)
    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            node_name='joint_state_publisher_collada',
            arguments=[os.path.join(test_urdfs_dir, 'multi_joint_robot.dae')],
            remappings=[('joint_states', 'joint_states/collada')]),
        Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            node_name='joint_state_publisher_urdf',
            arguments=[os.path.join(test_urdfs_dir, 'multi_joint_robot.urdf')],
            remappings=[('joint_states', 'joint_states/urdf')]),
        launch_testing.actions.ReadyToTest(),
    ])


class TestMultiJoint(unittest.TestCase):
    TIMEOUT = 10

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_multi_joint')

    def tearDown(self):
        self.node.destroy_node()

    def test_joints_published(self):
        msgs_rx_collada = []
        sub_collada = self.node.create_subscription(
            sensor_msgs.msg.JointState, 'joint_states/collada',
            lambda msg: msgs_rx_collada.append(msg), 1)
        msgs_rx_urdf = []
        sub_urdf = self.node.create_subscription(
            sensor_msgs.msg.JointState, 'joint_states/urdf',
            lambda msg: msgs_rx_urdf.append(msg), 1)
        try:
            end_time = time.monotonic() + self.TIMEOUT
            while time.monotonic() < end_time and (not msgs_rx_collada or not msgs_rx_urdf):
                rclpy.spin_once(self.node, timeout_sec=0.1)

            assert msgs_rx_collada
            assert msgs_rx_urdf
            for msg in msgs_rx_collada + msgs_rx_urdf:
                assert 2 == len(msg.name)
                assert 2 == len(msg.position)
                assert 0 == len(msg.velocity)
                assert 0 == len(msg.effort)
                assert {'j12': 0.0, 'j23': 0.0} == dict(zip(msg.name, msg.position))
        finally:
            self.node.destroy_subscription(sub_collada)
            self.node.destroy_subscription(sub_urdf)
