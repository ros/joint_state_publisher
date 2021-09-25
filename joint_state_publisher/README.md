# Joint State Publisher

This contains a package for publishing `sensor_msgs/JointState` messages for a robot described with URDF.
Given a URDF in the `robot_description` parameter on the parameter server, this node
will continually publish default values for all of the movable joints in the URDF to the `/joint_states` topic.

See the ROS wiki for additional API documentation and tutorials.

* [`joint_state_publisher`](http://wiki.ros.org/joint_state_publisher)

This was originally part of the [`ros/robot_model`](https://github.com/ros/robot_model) repository.
It has been moved to this repo as described by [`ros/robot_model#195`](https://github.com/ros/robot_model/issues/195)

Published Topics
----------------
* `/joint_states` (`sensor_msgs/JointState`) - The state of all of the movable joints in the system.

Subscribed Topics
-----------------
* (optional) `/any_topic` (`sensor_msgs/JointState`) - If the `sources_list` parameter is not empty (see Parameters below), then every named topic in this parameter will be subscribed to for joint state updates.  Do *not* add the default `/joint_states` topic to this list, as it will end up in an endless loop!

Parameters
----------
* `robot_description` (string, required) - A URDF or DAE file describing the robot.
* `rate` (int) - The rate at which to publish updates to the `/joint_states` topic.  Defaults to 10.
* `publish_default_positions` (bool) - Whether to publish a default position for each movable joint to the `/joint_states` topic.  Defaults to True.
* `publish_default_velocities` (bool) - Whether to publish a default velocity for each movable joint to the `/joint_states` topic.  Defaults to False.
* `publish_default_efforts` (bool) - Whether to publish a default effort for each movable joint to the `/joint_states` topic.  Defaults to False.
* `use_mimic_tags` (bool) - Whether to honor `<mimic>` tags in the URDF.  Defaults to True.
* `use_smallest_joint_limits` (bool) - Whether to honor `<safety_controller>` tags in the URDF.  Defaults to True.
* `source_list` (array of strings) - Each string in this array represents a topic name.  For each string, create a subscription to the named topic of type `sensor_msgs/JointStates`.  Publication to that topic will update the joints named in the message.  Defaults to an empty array.
* `zeros` (dictionary of string -> float) - A dictionary of joint_names to initial starting values for the joint.  Defaults to an empty dictionary, in which case 0.0 is assumed as the zero for all joints.
* `dependent_joints` (dictionary of string -> dictionary of 'parent', 'factor', 'offset') - A dictionary of joint_names to the joints that they mimic; compare to the `<mimic>` tag in URDF.  A joint listed here will mimic the movements of the 'parent' joint, subject to the 'factor' and 'offset' provided.  The 'parent' name must be provided, while the 'factor' and 'offset' parameters are optional (they default to 1.0 and 0.0, respectively).  Defaults to the empty dictionary, in which case only joints that are marked as `<mimic>` in the URDF are mimiced.
