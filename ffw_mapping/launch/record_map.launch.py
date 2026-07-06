import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # We want to save the rosbag inside the source directory of the package
    # so it persists across builds and is easy to find.
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    pkg_src_dir = os.path.expanduser(f'~/robotis_ws/src/ai_worker/ffw_mapping/scan_{timestamp}')

    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', pkg_src_dir, '/scan', '/ffw_laser_odom'],
        output='screen'
    )

    return LaunchDescription([
        record_process
    ])
