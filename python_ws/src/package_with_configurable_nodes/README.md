# Launch_files - Parameters - Runtime_configuration
This package contains programs that demonstrate:
- The use of launch files to launch multiple nodes in parallel
- The use of launch files to configure a node at spinoff
- The runtime configurability of nodes
- ROS2 Param

A parameters could have as well been a variable in the node, except that
the value it holds can be changed when the node is running. In other words,
they make the node configurable in runtime<br>

You first declare them, then set them, the you can repeatedly set and retrieve
them. Parameters are decleared in the __init__ method.

A launch file can also set a parameter for a node at startup
