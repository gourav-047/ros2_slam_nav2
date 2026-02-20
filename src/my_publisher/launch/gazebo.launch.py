from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('my_publisher')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'robot.rviz','worlds','maze_world.world')
    world_file = os.path.join(pkg_share,'worlds','maze_world.world')


    # Gazebo  
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')), 
      
       launch_arguments={
            'world': world_file,   
            'verbose': 'true'
        }.items()
    )



    # Robot State Publisher 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': open(urdf_file).read()},
            {'use_sim_time': True}
        ],
        output='screen'
    )
     
    # joint_state_publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # Spawn Robot 
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    # # RViz 
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

  # TELEOP 
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',   
        output='screen'

    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # rviz
    ])
