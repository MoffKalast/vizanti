import launch
import launch_ros.actions

def generate_launch_description():
    retry_startup_delay = launch.substitutions.LaunchConfiguration('retry_startup_delay', default='10.0')
    fragment_timeout = launch.substitutions.LaunchConfiguration('fragment_timeout', default='30')
    delay_between_messages = launch.substitutions.LaunchConfiguration('delay_between_messages', default='0')
    max_message_size = launch.substitutions.LaunchConfiguration('max_message_size', default='10000000')
    unregister_timeout = launch.substitutions.LaunchConfiguration('unregister_timeout', default='100.0')

    rosbridge_node = launch_ros.actions.Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[
            {'authenticate': False},
            {'port': 5001},
            {'address': ''},
            {'retry_startup_delay': retry_startup_delay},
            {'fragment_timeout': fragment_timeout},
            {'delay_between_messages': delay_between_messages},
            {'max_message_size': max_message_size},
            {'unregister_timeout': unregister_timeout}
        ]
    )

    rosapi_node = launch_ros.actions.Node(
        package='rosapi',
        executable='rosapi_node'
    )

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
        rosbridge_node,
        rosapi_node,
        flask_node,
        topic_handler_node,
        service_handler_node
    ])

if __name__ == '__main__':
    launch.main()
