# AIDDL Bridge for ROS2

A collection of ROS2 nodes that work as bridges to the AIDDL framework for
integrative AI.

## How does it work?

There are four generic ROS2 nodes that can be configured by providing the concrete ROS classes and converters:

- Sender: Publish AIDDL terms to a ROS2 topic
- Receiver: Subscribe to a ROS2 topic and read its messages as AIDDL terms
- Service: Call a ROS2 service with an AIDDL term and convert the answer back to
  an AIDDL term
- Actor: send AIDDL term as goal to an action server, read feedback and results
  as AIDDL terms

Each node works both as a ROS2 node and as a gRPC server. The AIDDL framework's
gRPC libraries (`aiddl_external_grpc`) can then be used to communicate with ROS2
without directly requiring a ROS2 library. 

Nodes are configured with converter classes, ROS2 topics, and gRPC
ports. Converters are loaded dynamically and are provided for standard ROS2
messages, actions, and services in the `aiddl_external_ros2` python library. For
custom messages, small custom converter libraries can be created.

## Installation and Setup

1. Clone this repository into the `src` folder of a ROS2 workspace.
2. Install `aiddl_external_ros2` (provides converters and utility used by `aiddl_ros2_bridge`)

        pip install aiddl-external-ros2

3. Source ROS2 Humble:

        source /opt/ros/humble/setup.bash
        
4. Build the `aiddl_ros2_bridge` package. In the ROS2 workspace folder, run:

        colcon build --package-select aiddl_ros2_bridge
        
5. Source the resulting package: 
        
        source install/setup.bash 

After these steps are completed, you can write YAML files to configure ROS2 nodes and launch them without 
any additional coding (assuming you use converters already provided by `aiddl_external_ros2`) 

## ROS Parameters

Nodes are configured with a set of ROS parameters. 

- `grpc_port` port on which the gRPC server will be opened
- `ros_topic` name of ROS2 topic related to publisher, subscriber, action, or service
- `ros_class` name of relevant ROS2 class (either message, action, or service)
  - This sets `<ROS_CLASS>` below
- `converter_class` class for a converter between AIDDL and ROS2. 
  - This class must be visible to whichever Python environment is running the ROS2 node.
  - The `aiddl_external_ros2` Python library provides a set of converters for standard ROS2 messages
  - Depending on the type of node different **static** methods are expected 
    - Sender: 
      - `aiddl2ros(term: Term) -> <ROS_CLASS>`
    - Receiver: 
      - `ros2aiddl(msg: <ROS_CLASS>) -> Term`
    - Action: 
      - `request_aiddl2ros(term: Term) -> <ROS_CLASS>.Goal()`
      - `feedback_ros2aiddl(msg: <ROS_CLASS>.Feedback())`
      - `result_ros2aiddl(msg: <ROS_CLASS>.Result())` 
    - Service: 
      - `request_aiddl2ros(term: Term) -> <ROS_CLASS>.Request()`
      - `result_ros2aiddl(msg: <ROS_CLASS>.Result()) -> Term`

## Examples

Each of the following examples creates a node of a different type and assumes that `aiddl_external_ros2` is installed. 

### Example: Sender Node for Visualization Markers
     
1. Create a file `test-params.yaml` with the following content:

        my_marker_sender:
            ros__parameters:
                grpc_port: 8066
                ros_topic: visualization_marker
                converter_class: "aiddl_external_ros2.converter.visualization_msgs_marker.MarkerConverter"
                ros_msg_class: "visualization_msgs.msg.Marker"

2. Run the node using the following command:

        ros2 run aiddl_ros2_bridge sender --ros-args -r __node:=my_marker_sender --params-file test_params.yaml

### Example: Receiver Node for Robot Poses

1. Create a file `test-params.yaml` with the following content:

        my_pose_receiver:
            ros__parameters:
                grpc_port: 8067
                ros_topic: /robot1/pose
                converter_class: "aiddl_external_ros2.converter.PoseStampedConverter"
                ros_msg_class: "geometry_msgs.msg.PoseStamped"

2. Run the node with: 

        ros2 run aiddl_ros2_bridge receiver --ros-args -r __node:=my_pose_receiver --params-file test_params.yaml

### Example: Moving a Robot with Nav2

1. Create a file `test-params.yaml` with the following content (or attach to the previous file):

        my_nav2_move_robot1_to_pose_client:
            ros__parameters:
                grpc_port: 8065
                ros_topic: /robot1/navigate_to_pose
                converter_class: "aiddl_external_ros2.converter.action.NavigateToPoseConverter"
                ros_action_class: "nav2_msgs.action.NavigateToPose"

2. Run the node with:

        ros2 run aiddl_ros2_bridge receiver --ros-args -r __node:=my_pose_receiver --params-file test_params.yaml

## Example: Using a ROS2 Launch File

Connecting AIDDL to ROS2 may require many nodes and starting them individually is not practical.
In this case it is easiest to write a ROS2 launch file. This contains essentially the same information as
the YAML files above, but also specifies the package, executable, name, and namespace. As a result this information
is not needed on the command-line level anymore.

For brevity, we include only a single node in the following example. 

1. Create a file `my_launch_file.py` and add the following lines:

        from launch import LaunchDescription
        from launch.actions import DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration
        from launch_ros.actions import Node

        def generate_launch_description():
           return LaunchDescription([
              Node(
                 package='aiddl_ros2_bridge',
                 executable='actor',
                 name='move',
                 namespace='robot1',
                 parameters=[
                    {
                        'grpc_port': 8065,
                        'ros_topic': '/robot1/navigate_to_pose',
                        'converter_class': 'aiddl_external_ros2.action.nav2.NavigateToPoseConverter',
                        'ros_class': 'nav2_msgs.action.NavigateToPose'
                    }
                 ]
              ),
           ])

2. Run all nodes in the file with:

        ros2 launch my_launch_file.py

## List of Converters

To avoid maintaining information twice, we refer to the documentation of the `aiddl-external-ros2` Python package
which can be found here.

**TODO:** Add link to `aiddl_external_ros2` README detailing available converter class names.

## How to Create Custom Converters

If your project uses custom actions, services, or message types, you can write your own converters.

1. Create a new Python project
2. Add requirements for `aiddl-core` and `aiddl-external-ros2`
3. Create one class for each message, action, or service according to the interface detailed above
4. Locally install your project so the `aiddl_ros2_bridge` package can see the converter classes when bringing up nodes

        pip install -e .

**TODO:** Provide example/skeleton for custom messages.