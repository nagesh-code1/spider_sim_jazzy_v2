"""spider_sim: ROS 2 Jazzy + Gazebo Sim (Harmonic).

    ros2 launch spider_sim sim.launch.py                       # Gazebo window
    ros2 launch spider_sim sim.launch.py headless:=true        # no window
    ros2 launch spider_sim sim.launch.py headless:=true record:=true record_dir:=$HOME/spider_rec

FIXES vs the original: no hard-coded /opt/ros path, /clock is bridged so
use_sim_time works, all nodes run on sim time, controllers and nodes start
after the robot is spawned (instead of fixed timers), optional recording.
"""
import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_TIME = {'use_sim_time': True}


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def _world_file(pkg, record, rec_dir):
    with open(os.path.join(pkg, 'worlds', 'spider_world.sdf')) as f:
        xml = f.read().replace('__RECORD__', 'true' if record else 'false') \
                      .replace('__RECDIR__', rec_dir)
    path = os.path.join(tempfile.gettempdir(), 'spider_world_run.sdf')
    with open(path, 'w') as f:
        f.write(xml)
    return path


def _setup(context):
    pkg = get_package_share_directory('spider_sim')
    headless = _flag(context, 'headless')
    record = _flag(context, 'record')
    rec_dir = os.path.expanduser(LaunchConfiguration('record_dir').perform(context))
    if record:
        for sub in ('overview', 'chase'):
            os.makedirs(os.path.join(rec_dir, sub), exist_ok=True)

    world = _world_file(pkg, record, rec_dir)
    description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'spider.urdf.xacro'),
        mappings={'record': 'true' if record else 'false', 'record_dir': rec_dir}).toxml()

    gz_args = f'-r -v 2 {world}'
    if headless:
        gz_args = f'-r -s --headless-rendering -v 2 {world}'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': description}, SIM_TIME], output='screen')

    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'spider', '-topic', 'robot_description',
                            '-x', '0', '-y', '0', '-z', '0.12'],
                 output='screen')

    topics = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    if record:
        # gz only renders a camera that has a subscriber, so bridge them while recording
        topics += ['/overview_cam@sensor_msgs/msg/Image[gz.msgs.Image',
                   '/chase_cam@sensor_msgs/msg/Image[gz.msgs.Image']
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=topics, parameters=[SIM_TIME], output='screen')

    controllers = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', 'leg_controller', 'arm_controller',
                                  '--controller-manager', '/controller_manager',
                                  # with cameras rendering the sim runs slowly, so the
                                  # default 5 s switch timeout is too short
                                  '--switch-timeout', '60',
                                  '--controller-manager-timeout', '120'],
                       output='screen')

    nodes = [Node(package='spider_sim', executable=exe, parameters=[SIM_TIME], output='screen')
             for exe in ('gait_controller', 'arm_ik', 'gripper_controller', 'mission_manager')]

    return [
        gazebo, rsp, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers])),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=nodes)),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='run the Gazebo server only (no window)'),
        DeclareLaunchArgument('record', default_value='false',
                              description='save overview + chase camera frames as PNG'),
        DeclareLaunchArgument('record_dir', default_value='~/spider_rec_v2'),
        OpaqueFunction(function=_setup),
    ])
