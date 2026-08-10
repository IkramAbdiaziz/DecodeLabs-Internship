import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
  # 1. Paths to packages and files
  pkg_share = get_package_share_directory('robotic_arm_kinematics')
  gazebo_share = get_package_share_directory('gazebo_ros')

  xacro_file = os.path.join(pkg_share, 'urdf', 'robot_arm.urdf.xacro')
  rviz_config = os.path.join(pkg_share, 'config', 'simulation.rviz')

  # 2. Process Xacro to load robot_description
  robot_description_content = Command(['xacro ', xacro_file])
  robot_description = {'robot_description': robot_description_content}

  # Minimal SRDF semantic description
  robot_description_semantic_content = (
      '<?xml version="1.0" encoding="UTF-8"?><robot name="robotic_arm">'
      '<group name="arm">'
      '<joint name="joint1"/><joint name="joint2"/><joint name="joint3"/>'
      '<joint name="joint4"/><joint name="joint5"/><joint name="joint6"/>'
      '</group></robot>'
  )
  robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

  # 3. Robot State Publisher Node
  robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[robot_description]
  )

  # 4. Gazebo simulation client/server
  gazebo_server = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
      )
  )
  
  gazebo_client = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')
      )
  )

  # 5. Spawn entity in Gazebo
  spawn_entity = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-topic', 'robot_description', '-entity', 'robotic_arm', '-z', '0.0'],
      output='screen'
  )

  # 6. ROS 2 Controllers spawners (triggered after spawn_entity finishes)
  joint_state_broadcaster_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['joint_state_broadcaster'],
      output='screen'
  )

  arm_controller_spawner = Node(
      package='controller_manager',
      executable='spawner',
      arguments=['arm_controller'],
      output='screen'
  )

  # Delay loading of controllers until after spawning is finished to prevent controller manager crashes
  load_joint_state_broadcaster = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=spawn_entity,
          on_exit=[joint_state_broadcaster_spawner],
      )
  )

  load_arm_controller = RegisterEventHandler(
      event_handler=OnProcessExit(
          target_action=joint_state_broadcaster_spawner,
          on_exit=[arm_controller_spawner],
      )
  )

  # 7. RViz2 Display Node
  rviz = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config],
      output='screen'
  )

  # 8. Kinematics Planner Node (requires robot_description and robot_description_semantic parameters)
  planner_node = Node(
      package='robotic_arm_kinematics',
      executable='kinematics_planner_node',
      output='screen',
      parameters=[
          robot_description,
          robot_description_semantic,
          {
              'base_link': 'base_link',
              'end_effector': 'end_effector',
              'solve_timeout': 0.010,
              'solve_epsilon': 1e-5,
              'trajectory_frequency': 20.0
          }
      ]
  )

  return LaunchDescription([
      gazebo_server,
      gazebo_client,
      robot_state_publisher,
      spawn_entity,
      load_joint_state_broadcaster,
      load_arm_controller,
      rviz,
      planner_node
  ])
