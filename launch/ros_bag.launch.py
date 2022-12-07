from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    
    all_topics = LaunchConfiguration('all_topics')

    return LaunchDescription([

        DeclareLaunchArgument(
            'all_topics',
            default_value='False'
        ),

        Node(
            package='ros2_roomba',
            executable='redirect_from_obstacles',
            name='roomba_walker',
            parameters=[{
                "all_topics": LaunchConfiguration('all_topics'),
            }]
        ),

        ExecuteProcess(
        condition=IfCondition(all_topics),
        cmd=[
            'ros2', 'bag', 'record', '-o results/saved_bag', '-a', '-x', '/scan.*'
        ],
        shell=True
        )

    ])