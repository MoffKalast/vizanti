import launch
import launch_ros.actions

def generate_launch_description():
    base_url = launch.substitutions.LaunchConfiguration('base_url', default='') #e.g. /vizanti
    port = launch.substitutions.LaunchConfiguration('port', default=5010)
    port_rosbridge = launch.substitutions.LaunchConfiguration('port_rosbridge', default=5011)
    flask_debug = launch.substitutions.LaunchConfiguration('flask_debug', default=True)

    #https://github.com/v-kiniv/rws
    rws_server_node = launch_ros.actions.Node(
        package='rws',
        executable='rws_server',
        name='vizanti_rws_server',
        output='screen',
        parameters=[
            {'rosbridge_compatible ': True},
            {'port': port_rosbridge},
            {'watchdog': True}
        ]
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
            {'compression': "cbor"}
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
        rws_server_node,
        flask_node,
        tf_handler_node,
        service_handler_node
    ])

if __name__ == '__main__':
    launch.main()
