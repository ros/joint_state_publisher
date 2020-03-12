# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math
import sys
import xml.dom.minidom

import rospy
import sensor_msgs.msg


def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value


class JointStatePublisher():
    def init_collada(self, robot):
        robot = robot.getElementsByTagName('kinematics_model')[0].getElementsByTagName('technique_common')[0]
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                name = child.getAttribute('name')
                if child.getElementsByTagName('revolute'):
                    joint = child.getElementsByTagName('revolute')[0]
                else:
                    rospy.logwarn("Unknown joint type %s", child)
                    continue

                if joint:
                    limit = joint.getElementsByTagName('limits')[0]
                    minval = float(limit.getElementsByTagName('min')[0].childNodes[0].nodeValue)
                    maxval = float(limit.getElementsByTagName('max')[0].childNodes[0].nodeValue)
                if minval == maxval:  # this is fixed joint
                    continue

                self.joint_list.append(name)
                joint = {'min':minval*math.pi/180.0, 'max':maxval*math.pi/180.0, 'zero':0, 'position':0, 'velocity':0, 'effort':0}
                self.free_joints[name] = joint

    def init_urdf(self, robot):
        robot = robot.getElementsByTagName('robot')[0]
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype in ['fixed', 'floating', 'planar']:
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -math.pi
                    maxval = math.pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if self.use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if self.use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min': minval, 'max': maxval, 'zero': zeroval}
                if self.pub_def_positions:
                    joint['position'] = zeroval
                if self.pub_def_vels:
                    joint['velocity'] = 0.0
                if self.pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint

    def __init__(self):
        description = get_param('robot_description')
        if description is None:
            raise RuntimeError('The robot_description parameter is required and not set.')

        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        self.use_mimic = get_param('use_mimic_tags', True)
        self.use_small = get_param('use_smallest_joint_limits', True)

        self.zeros = get_param("zeros")

        self.pub_def_positions = get_param("publish_default_positions", True)
        self.pub_def_vels = get_param("publish_default_velocities", False)
        self.pub_def_efforts = get_param("publish_default_efforts", False)

        robot = xml.dom.minidom.parseString(description)
        if robot.getElementsByTagName('COLLADA'):
            self.init_collada(robot)
        else:
            self.init_urdf(robot)

        # The source_update_cb will be called at the end of self.source_cb.
        # The main purpose it to allow external observes (such as the
        # joint_state_publisher_gui) to be notified when things are updated.
        self.source_update_cb = None

        source_list = get_param("source_list", [])
        self.sources = []
        for source in source_list:
            self.sources.append(rospy.Subscriber(source, sensor_msgs.msg.JointState, self.source_cb))

        self.pub = rospy.Publisher('joint_states', sensor_msgs.msg.JointState, queue_size=5)

    def source_cb(self, msg):
        for i in range(len(msg.name)):
            name = msg.name[i]
            if name not in self.free_joints:
                continue

            if msg.position:
                position = msg.position[i]
            else:
                position = None
            if msg.velocity:
                velocity = msg.velocity[i]
            else:
                velocity = None
            if msg.effort:
                effort = msg.effort[i]
            else:
                effort = None

            joint = self.free_joints[name]
            if position is not None:
                joint['position'] = position
            if velocity is not None:
                joint['velocity'] = velocity
            if effort is not None:
                joint['effort'] = effort

        if self.source_update_cb is not None:
            self.source_update_cb()

    def set_source_update_cb(self, user_cb):
        self.source_update_cb = user_cb

    def loop(self):
        hz = get_param("rate", 10)  # 10hz
        r = rospy.Rate(hz)

        delta = get_param("delta", 0.0)

        # Publish Joint States
        while not rospy.is_shutdown():
            msg = sensor_msgs.msg.JointState()
            msg.header.stamp = rospy.Time.now()

            if delta > 0:
                self.update(delta)

            # Initialize msg.position, msg.velocity, and msg.effort.
            has_position = len(self.dependent_joints.items()) > 0
            has_velocity = False
            has_effort = False
            for name, joint in self.free_joints.items():
                if not has_position and 'position' in joint:
                    has_position = True
                if not has_velocity and 'velocity' in joint:
                    has_velocity = True
                if not has_effort and 'effort' in joint:
                    has_effort = True
            num_joints = (len(self.free_joints.items()) +
                          len(self.dependent_joints.items()))
            if has_position:
                msg.position = num_joints * [0.0]
            if has_velocity:
                msg.velocity = num_joints * [0.0]
            if has_effort:
                msg.effort = num_joints * [0.0]

            for i, name in enumerate(self.joint_list):
                msg.name.append(str(name))
                joint = None

                # Add Free Joint
                if name in self.free_joints:
                    joint = self.free_joints[name]
                    factor = 1
                    offset = 0
                # Add Dependent Joint
                elif name in self.dependent_joints:
                    param = self.dependent_joints[name]
                    parent = param['parent']
                    factor = param.get('factor', 1)
                    offset = param.get('offset', 0)
                    # Handle recursive mimic chain
                    recursive_mimic_chain_joints = [name]
                    while parent in self.dependent_joints:
                        if parent in recursive_mimic_chain_joints:
                            error_message = "Found an infinite recursive mimic chain"
                            rospy.logerr("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
                            sys.exit(1)
                        recursive_mimic_chain_joints.append(parent)
                        param = self.dependent_joints[parent]
                        parent = param['parent']
                        offset += factor * param.get('offset', 0)
                        factor *= param.get('factor', 1)
                    joint = self.free_joints[parent]

                if has_position and 'position' in joint:
                    msg.position[i] = joint['position'] * factor + offset
                if has_velocity and 'velocity' in joint:
                    msg.velocity[i] = joint['velocity'] * factor
                if has_effort and 'effort' in joint:
                    msg.effort[i] = joint['effort']

            if msg.name or msg.position or msg.velocity or msg.effort:
                # Only publish non-empty messages
                self.pub.publish(msg)
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass

    def update(self, delta):
        for name, joint in self.free_joints.iteritems():
            forward = joint.get('forward', True)
            if forward:
                joint['position'] += delta
                if joint['position'] > joint['max']:
                    if joint.get('continuous', False):
                        joint['position'] = joint['min']
                    else:
                        joint['position'] = joint['max']
                        joint['forward'] = not forward
            else:
                joint['position'] -= delta
                if joint['position'] < joint['min']:
                    joint['position'] = joint['min']
                    joint['forward'] = not forward
