import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Package Paths
    pkg_share = get_package_share_directory('amr_navigation')
    gazebo_share = get_package_share_directory('gazebo_ros')

    # 2. Config File Paths
    xacro_file = os.path.join(pkg_share, 'urdf', 'amr_robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'config', 'maze.world')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'simulation.rviz')

    # 3. Process Robot Description Xacro
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # 4. Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # 5. Gazebo Server (launches the custom maze world)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # 6. Gazebo Client (GUI renderer)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')
        )
    )

    # 7. Spawn Robot Entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'amr_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )

    # 8. Robot State Estimation EKF Node (Fused sensor-level odometry)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}]
    )

    # 9. Asynchronous SLAM Toolbox Node (Generates 2D occupancy grid mapping)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}]
    )

    # 10. RViz2 Visualization Tool (with pre-loaded topic displays)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 11. Custom A* Pathfinder & Lookahead Follower Node
    astar_node = Node(
        package='amr_navigation',
        executable='astar_planner_node',
        name='astar_planner_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 12. Reflex-Level Dynamic Deceleration Obstacle Avoidance Override Filter
    avoidance_node = Node(
        package='amr_navigation',
        executable='dynamic_avoidance_node',
        name='dynamic_avoidance_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Simulation
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,
        
        # State Estimation & Mapping
        ekf_node,
        slam_toolbox,
        
        # UI Visualizations
        rviz,
        
        # Custom Autonomous Steering Nodes
        astar_node,
        avoidance_node
    ])
