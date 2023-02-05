---
layout: single
title: Giving a TurtleBot3 a Namespace for Multi-Robot Experiments
categories: [general, guide]
tags: [ros2, turtlebot]
fullview: true
comments: false
classes: wide
---

As I was working on my ICRA paper, I noticed that ROBOTIS doesn't provided a guide on how to run multiple TurtleBot3 robots together. It is especially dangerous if you run them in the same network because they all run on the same topic names and node names, which can interfere with their individual operation. So to help run multiple TurtleBots on the same network, you need to give each robot a unique namespace. The following guide will show you how to do this for the TurtleBot3.

For this guide, we will be using `tb3_0` as the namespace we wish to use for our TurtleBot3 Burger robot. This helps us number our robots easier when running multiple robot experiments. This guide also assumes you have followed the procedure located [here](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/#sbc-setup) for installing and setting up your TurtleBot3 with ROS 2!

&nbsp;

### Step 1: Create a New ROS 2 Package

Start by changing into your `src` directory of your workspace that also contains the `turtlebot3` and `utils` packages provided by ROBOTIS.

```bash
~$ cd ~/turtlebot3_ws/src
~$ ros2 pkg create my_tb3_launcher
```

Now, create two empty directories in the new package:

```bash
~$ cd ~/turtlebot3_ws/src/my_tb3_launcher
~$ mkdir launch
~$ mkdir param
```

Change into the `launch` directory and create a new bringup launch file.

```bash
~$ cd launch
~$ touch my_tb3_bringup.launch.py
```

&nbsp;

### Step 2: Copy and Modify Contents from the TB3 Bringup Package into Your Package

In the `turtlebot3/turtlebot3_bringup` ROS 2 package, copy the contents of `robot.launch.py` into the `my_tb3_bringup.launch.py` with the following changes marked as `# comments` in the following code:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('my_tb3_launcher'),  # <--- CHANGE THIS!
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hlds_laser.launch.py']),  <--- CHANGE THIS
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='turtlebot3_node',
            node_executable='turtlebot3_ros',
            node_namespace='tb3_0',  # <------------------- ADD THIS!
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])

```

Next, copy the file `turtlebot3_state_publisher.launch.py` from the `turtlebot3_bringup/launch` directory into your package's `launch` directory. Make sure it has the same name! Once complete, make the following changes as marked by the following comments:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            node_namespace='tb3_0',  # <------------------- ADD THIS!
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),
    ])

```

Finally, copy the file `hlds_laser.launch.py` from the `hls_lfcd_lds_driver` package located in the `launch` directory into your package's `launch` directory. Again, make sure it has the same name!. Modify the launch file with the following changes marked by the comments below:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')

    frame_id = LaunchConfiguration('frame_id', default='laser')

    return LaunchDescription([

        DeclareLaunchArgument(
            'port',
            default_value=port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar. Default frame_id is \'laser\''),

        Node(
            package='hls_lfcd_lds_driver',
            node_executable='hlds_laser_publisher',
            node_name='hlds_laser_publisher',
            node_namespace='tb3_0',  # <------------------- ADD THIS!
            parameters=[{'port': port, 'frame_id': frame_id}],
            output='screen'),
    ])
```

&nbsp;

### Step 3: Modify the Parameter YAML File

Now copy the `burger.yaml` file located in the `param` directory of the `turtlebot3_bringup` package, and make the following modification at the top!

```yaml
tb3_0:
  turtlebot3_node:
    ros__parameters:

      opencr:
        id: 200
        baud_rate: 1000000
        protocol_version: 2.0

      wheels:
        separation: 0.160
        radius: 0.033

      motors:
        profile_acceleration_constant: 214.577

        # [rev/min2]
        # ref) http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#profile-acceleration
        profile_acceleration: 0.0

      sensors:
        bumper_1: false
        bumper_2: false

        illumination: false

        ir: false

        sonar: false

tb3_0:
  diff_drive_controller:
    ros__parameters:

      odometry:
        publish_tf: true
        use_imu: true
        frame_id: "odom"
        child_frame_id: "base_footprint"
```

As you can see, the top most parameter used to be the node name (`turtlebot3_node` and `diff_drive_controller`). For the node namespace that you added to work, you will need to add the node namespace (`tb3_0`) one level *above* the node name!

In Step 2, we already changed the launch file to point to *this* yaml file instead of the one located in the `turtlebot3_bringup` package.

&nbsp;

### Step 4: Modify the CMakeLists File

For this section, we will just be adding a small code snipet to our `CMakeLists.txt` that will install the `launch` and `param` contents of our `my_tb3_launcher` package.

```bash
...
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)
...
```

Add this snippet right before the `if(BUILD_TESTING)` section of the `CMakeLists.txt` file.

&nbsp;

### Step 5: Compile and Run

Finally, compile the code on your TurtleBot3:

```bash
~$ cd ~/turtlebot3_ws
~$ colcon build --symlink-install --parallel-workers 1
~$ . install/setup.bash
```

Now, run your launch file to make sure it works!

```bash
~$ export TURTLEBOT3_MODEL=burger
~$ ros2 launch my_tb3_launcher my_tb3_bringup.launch.py
```

You should get the following topics when you run `ros2 topic list` in another bash session:

```bash
/tb3_0/battery_state
/tb3_0/cmd_vel
/tb3_0/imu
/tb3_0/joint_states
/tb3_0/magnetic_field
/tb3_0/odom
/tb3_0/parameter_events
/tb3_0/robot_description
/tb3_0/rosout
/tb3_0/scan
/tb3_0/sensor_state
/tb3_0/tf
/tb3_0/tf_static
```

You can repeat these procedures with other TurtleBot3 robots with different namespaces to have multiple robots working in your network. Hope this helps and happy programming everyone!
