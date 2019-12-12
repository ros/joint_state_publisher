# Joint State Publisher

This contains a package for publishing `sensor_msgs/JointState` messages for a robot described with URDF.

Given a URDF (either passed on the command-line or via the `/robot_description` topic), this node
will continually publish default values for all of the movable joints in the URDF to the `/joint_states` topic.
If the `use_gui` parameter is True, it will also launch a window where the values of the movable joints can be updated.
In combination with `robot_state_publisher`, this can update the TF2 transforms for the robot on the fly.

Published Topics
----------------
* `/joint_states` (`sensor_msgs/JointState`) - The state of all of the movable joints in the system.

Subscribed Topics
-----------------
* (optional) `/robot_description` (`std_msgs/String`) - If no URDF is given on the command-line, then this node will listen on the `/robot_description` topic for the URDF to be published.  Once it has been received at least once, this node will start to publish joint values to `/joint_states`.
* (optional) `/any_topic` (`sensor_msgs/JointState`) - If the `sources_list` parameter is not empty (see Parameters below), then every named topic in this parameter will be subscribed to for joint state updates.  Do *not* add the default `/joint_states` topic to this list, as it will end up in an endless loop!

Parameters
----------
* `rate` (int) - The rate at which to publish updates to the `/joint_states` topic.  Defaults to 10.
* `publish_default_positions` (bool) - Whether to publish a default position for each movable joint to the `/joint_states` topic.  Defaults to True.
* `publish_default_velocities` (bool) - Whether to publish a default velocity for each movable joint to the `/joint_states` topic.  Defaults to False.
* `publish_default_efforts` (bool) - Whether to publish a default effort for each movable joint to the `/joint_states` topic.  Defaults to False.
* `use_mimic_tags` (bool) - Whether to honor `<mimic>` tags in the URDF.  Defaults to True.
* `use_smallest_joint_limits` (bool) - Whether to honor `<safety_controller>` tags in the URDF.  Defaults to True.
* `source_list` (array of strings) - Each string in this array represents a topic name.  For each string, create a subscription to the named topic of type `sensor_msgs/JointStates`.  Publication to that topic will update the joints named in the message.  Defaults to an empty array.
* `use_gui` (bool) - If True, a GUI with sliders to control the movable joints will be launched when the node is launched.  Defaults to False.
* `num_rows` (int) - The number of rows to show in the GUI.  If 0 (the default), uses the number of movable joints as the number of rows.  Only used if `use_gui` is True.
