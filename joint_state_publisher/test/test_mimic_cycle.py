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
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.asserts
import pytest


@pytest.mark.rostest
def generate_test_description():
    test_urdfs_dir = os.path.dirname(__file__)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        arguments=[os.path.join(test_urdfs_dir, 'mimic_cycle.urdf')])
    return (
        LaunchDescription([joint_state_publisher, launch_testing.actions.ReadyToTest()]),
        {'joint_state_publisher': joint_state_publisher})


@launch_testing.post_shutdown_test()
class TestMimicCycleRobot(unittest.TestCase):

    def test_exit(self, proc_info, proc_output):
        jsp = list(proc_info.process_names())[0]
        # TODO(sloretz) Uncomment when ros2/launch#378 is released
        # launch_testing.asserts.assertInStderr(jsp, msg='Found an infinite recursive mimic chain')
        launch_testing.asserts.assertExitCodes(proc_info, process=jsp, allowable_exit_codes=[1])
