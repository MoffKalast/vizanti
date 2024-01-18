import launch
import launch_ros.actions

def generate_launch_description():
    base_url = launch.substitutions.LaunchConfiguration('base_url', default='') #e.g. /vizanti
    flask_debug = launch.substitutions.LaunchConfiguration('flask_debug', default=True)

    #https://github.com/v-kiniv/rws
    rws_server_node = launch_ros.actions.Node(
        package='rws',
        executable='rws_server',
        name='rws_server',
        output='screen',
        parameters=[
            {'rosbridge_compatible ': True},
            {'port': 5001},
            {'watchdog ': False}
        ]
    )

    flask_node = launch_ros.actions.Node(
        name='vizanti_flask_node',
        package='vizanti_server',
        executable='vizanti_flask_node',
        output='screen',
        parameters=[
            {'host': '0.0.0.0'},
            {'port': 5000},
            {'flask_debug': flask_debug},
            {'base_url': base_url}
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
        executable='vizanti_service_handler_node',
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
