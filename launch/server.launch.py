import launch
import launch_ros.actions

def generate_launch_description():
    flask_node = launch_ros.actions.Node(
        package='vizanti',
        executable='vizanti_flask_node',
        output='screen',
        parameters=[
            {'host': '0.0.0.0'},
            {'port': 5000},
            {'flask_debug': True}
        ]
    )

    topic_handler_node = launch_ros.actions.Node(
        package='vizanti',
        executable='vizanti_topic_handler_node',
        output='screen'
    )

    service_handler_node = launch_ros.actions.Node(
        package='vizanti',
        executable='vizanti_service_handler_node',
        output='screen'
    )

    return launch.LaunchDescription([
        flask_node,
        topic_handler_node,
        service_handler_node
    ])

if __name__ == '__main__':
    launch.main()
