
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ned_to_enu_pub',
            arguments=['0', '0', '0', '1.5707963', '0', '3.1415926', 'world_ned', 'world']
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_px4_odom_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'PX4_odom']
        )
    ])
    return ld
