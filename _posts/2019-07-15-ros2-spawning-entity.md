---
layout: single
title: Spawning Robots in Gazebo with ROS 2
categories: [general, demo]
tags: [ros2, vscode, macOS]
fullview: true
comments: false
classes: wide
---

Now that ROS 2 has done away with the old way of launching nodes (i.e. using XML `.launch` files), the process has become more stream-lined and versatile than ever before thanks to using Python. The new launch system can, however, be confusing to use the first time, and I'm probably going to do a deep-dive on it. For this blog post, I want to touch on something that is kind of missing from the old approach to the new one: spawning robots into Gazebo.

The old way of doing it was just to use an xml tag like this

{% highlight xml %}
<launch>
...
  <!-- URDF XML Pheeno robot description loaded on the parameter server. -->
  <param name="robot_description"
         command="$(find xacro)/xacro.py '$(find pheeno_ros_sim)/urdf/pheeno_v1/pheeno.xacro'"/>
  <!-- Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
        args="-urdf -model pheeno_01 -param robot_description -robot_namespace pheeno_01"/>
...
</launch>
{% endhighlight %}

As you can see (or remember if you have done this before), the old spawner is a node that can be called and provided arguments. I'm not sure what the main way of doing this in ROS 2 is, but according to Louise Poubel ([@chapulinaBR](https://twitter.com/chapulinaBR)) in this [ROS 2 migration guide](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete) it is kind of hinted that the official node approach is out. The best way to spawn a robot in Gazebo is to use a service call to `spawn_entity` for both urdf and sdf files. It looks something like this for sdf:

{% highlight bash %}
~$ ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' '{name: "sdf_ball", xml: "<?xml version=\"1.0\" ?><sdf version=\"1.5\"><model name=\"will_be_ignored\"><static>true</static><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>1.0</radius></sphere></geometry></visual></link></model></sdf>"}'
{% endhighlight %}

and this for urdf:

{% highlight bash %}
~$ ros2 service call /spawn_entity 'gazebo_msgs/SpawnEntity' '{name: "urdf_ball", xml: "<?xml version=\"1.0\" ?><robot name=\"will_be_ignored\"><link name=\"link\"><visual><geometry><sphere radius=\"1.0\"/></geometry></visual><inertial><mass value=\"1\"/><inertia ixx=\"1\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"1\" iyz=\"0.0\" izz=\"1\"/></inertial></link></robot>"}'
{% endhighlight %}

This is pretty straightforward, but one interesting thing about this is that the urdf or sdf file need to be given as the *xml* and not the *path* to the xml file. So if you have something a robot file that is super long this approach might not work for you.

An approach that does work, and if you are ok with using the `robot_state_publisher`, is to add the robot within the world file, like [this](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/ros2/turtlebot3_gazebo/worlds/empty_worlds/burger.model), and using a launch file to start it alongside gazebo like [this](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/ros2/turtlebot3_gazebo/launch/empty_world.launch.py). Both example files are from Robotis' ROS 2 simulation package located [here](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/ros2). This is a great resource for learning how to use ROS 2 using their platform.

&nbsp;

### Node for Spawning Your Own Entities

Instead of using a service call or generating a `.world` file containing the robot, you can create a node that can place robots in gazebo and use them in a launch file. For this example, I'm going to assume you have the turtlebot3 ROS 2 files [installed and setup](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/#setup) and have done the simulations section of the [guide](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/#simulation). To start, go into the `src` folder of the ROS 2 workspace that you created for the turtlebot3 (As an additional assumption, I will assume the workspace is the same name as the one created in the turtlebot3 guide):

```bash
~$ cd ~/turtlebot3_ws/src
~$ ros2 pkg create robot_spawner_pkg
```

Next, we are going to make this a python file only, so we will need to change delete the `CMakeLists.txt` file and create a `setup.py` and `setup.cfg` file containing the following:

{% highlight python linenos %}
from setuptools import setup

PACKAGE_NAME = 'robot_spawner_pkg'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    package_dir={'': 'src'}
    packages=[PACKAGE_NAME],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_turtlebot = robot_spawner_pkg.spawn_turtlebot:main',
        ],
    },
)
{% endhighlight %}

```bash
[develop]
script-dir=$base/lib/robot_spawner_pkg
[install]
install-scripts=$base/lib/robot_spawner_pkg
```

Now, we edit the `package.xml` file to add the necessary dependencies to run the program:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_controller_utils</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="your_email@email.com">your_name</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>rclpy</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Finally, we create our `spawn_turtlebot.py` file. Before we do that, however, we need to look at the current structure of our package.

```
robot_spawner_pkg/
    |
    ----include/
    |
    ---src/
    |
    ----setup.cfg
    ----setup.py
    ----package.xml
```

because in our `setup.py` file we defined the `package_dir={'': 'src'}` and the `packages=[PACKAGE_NAME]`, we need to make our package structure look like this:

```
robot_spawner_pkg/
    |
    ----include/
    |
    ---src/
    |  |
    |  ----robot_spawner_pkg/
    |
    ----setup.cfg
    ----setup.py
    ----package.xml
```

Within the second `robot_spwaner_pkg` folder will we be placing our `spawn_turtlebot.py` file!

&nbsp;

### The Robot Spawner Script

In this node, we will essentially be making the service call within the node like the following:

{% highlight python linenos %}
"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def main():
    """ Main for spwaning turtlebot node """
    # Get input arguments from user
    argv = sys.argv[1:]

    # Start node
    rclpy.init()
    sdf_file_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "models",
        "turtlebot3_burger", "model-1_4.sdf")
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the turtlebot3 burgerbot
    sdf_file_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "models",
        "turtlebot3_burger", "model-1_4.sdf")

    # Set data for request
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

{% endhighlight %}

I want to give credit to Dirk Thomas (can't seem to find the link to the code...) for this, because it is a modification of his code that inspired me to do this. Now build using `colcon`, source the install, and export the path to the turtlebot3 model like this:

```bash
~$ cd ~/turtlebot3_ws
~$ colcon build --symlink-install
~$ . install/setup.bash
~$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
```

To run the node, we first start a new terminal, source the installation, and start gazebo:

```bash
~$ cd ~/turtlebot3_ws
~$ . install/setup.bash
~$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
~$ gazebo --verbose -s libgazebo_ros_factory.so
```

Notice that we are using `libgazebo_ros_factory.so` instead of `libgazebo_ros_init.so` as used in the turtlebot3 and other tutorials. That is because only `libgazebo_ros_factory.so` contains the service call to `/spawn_entity`! Finally, back in your original terminal, use the following command to add a robot!

```bash
~$ ros2 run robot_spawner_pkg spawn_turtlebot the_robot_name robot_namespace 0.0 0.0 0.1
```

The first two arguments are the robot's name in gazebo and a namespace for the robot if you would like to add more than one. Hope that helps!
