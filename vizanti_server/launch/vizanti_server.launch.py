import launch
import launch_ros.actions

def generate_launch_description():

    #general params
    base_url = launch.substitutions.LaunchConfiguration('base_url', default='') #e.g. /vizanti
    port = launch.substitutions.LaunchConfiguration('port', default=5000)
    port_rosbridge = launch.substitutions.LaunchConfiguration('port_rosbridge', default=5001)
    flask_debug = launch.substitutions.LaunchConfiguration('flask_debug', default=True)
    default_widget_config = launch.substitutions.LaunchConfiguration('default_widget_config', default='') #e.g. ~/your_custom_config.json

    #rosbridge internal params
    unregister_timeout = launch.substitutions.LaunchConfiguration('unregister_timeout', default='9999999.9')
    retry_startup_delay = launch.substitutions.LaunchConfiguration('retry_startup_delay', default='10.0')
    fragment_timeout = launch.substitutions.LaunchConfiguration('fragment_timeout', default='30')
    delay_between_messages = launch.substitutions.LaunchConfiguration('delay_between_messages', default='0')
    max_message_size = launch.substitutions.LaunchConfiguration('max_message_size', default='999999999')

    rosbridge_node = launch_ros.actions.Node(
        name='vizanti_rosbridge',
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen',
        parameters=[
            {'authenticate': False},
            {'port': port_rosbridge},
            {'address': ''},
            {'retry_startup_delay': retry_startup_delay},
            {'fragment_timeout': fragment_timeout},
            {'delay_between_messages': delay_between_messages},
            {'max_message_size': max_message_size},
            {'unregister_timeout': unregister_timeout},
            {'use_compression': True}
        ]
    )

    rosapi_node = launch_ros.actions.Node(
        name='rosapi',
        package='rosapi',
        executable='rosapi_node'
    )

    flask_node = launch_ros.actions.Node(
        name='vizanti_flask_node',
        package='vizanti_server',
        executable='server.py',
        output='screen',
        parameters=[
            {'host': '0.0.0.0'},
            {'port': port},
            {'port_rosbridge': port_rosbridge},
            {'flask_debug': flask_debug},
            {'base_url': base_url},
            {'compression': "none"},
            {'default_widget_config': default_widget_config}
        ]
    )

    tf_handler_node = launch_ros.actions.Node(
        name='vizanti_tf_handler_node',
        package='vizanti_cpp',
        executable='tf_consolidator',
        output='screen'
    )

    service_handler_node = launch_ros.actions.Node(
        name='vizanti_service_handler_node',
        package='vizanti_server',
        executable='service_handler.py',
        output='screen'
    )

    return launch.LaunchDescription([
        rosbridge_node,
        rosapi_node,
        flask_node,
        tf_handler_node,
        service_handler_node
    ])

if __name__ == '__main__':
    launch.main()
