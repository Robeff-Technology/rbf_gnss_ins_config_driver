import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rbf_gnss_ins_config_driver'),
        'config',
        'gnss_ins_config.param.yaml'
    )

    container = ComposableNodeContainer(
    name='gnss_ins_config_driver_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='rbf_gnss_ins_config_driver',
            plugin='rbf_gnss_ins_config_driver::ConfigGnssIns',
            name='rbf_gnss_ins_config_driver',
            parameters=[config],
            extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ]
    )

    return launch.LaunchDescription([container])