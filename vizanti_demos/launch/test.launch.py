from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    test_grid_cells = Node(
        package='vizanti_demos',
        executable='test_grid_cells.py',
        name='test_grid_cells',
        output='screen',
        emulate_tty=True
    )

    test_marker_array = Node(
        package='vizanti_demos',
        executable='test_marker_array.py',
        name='test_marker_array',
        output='screen',
        emulate_tty=True
    )

    #Test transforms
    static_tf_publisher_node_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='test_tf_publisher_1',
        arguments=['10', '0', '0', '0', '0', '0', 'map', 'test_link'],
        output='screen'
    )

    static_tf_publisher_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='test_tf_publisher_2',
        arguments=['0', '10', '0', '0', '0', '0', 'test_link', 'test_child_link'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(test_grid_cells)
    ld.add_action(test_marker_array)
    ld.add_action(static_tf_publisher_node_1)
    ld.add_action(static_tf_publisher_node_2)
    return ld