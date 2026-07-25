import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # We want to save the rosbag inside the source directory of the package
    # so it persists across builds and is easy to find.
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    possible_dirs = [
        os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping'),
        '/root/ros2_ws/src/ai_worker/ffw_mapping',
        '/root/robotis_ws/src/ai_worker/ffw_mapping',
    ]
    target_dir = possible_dirs[0]
    for d in possible_dirs:
        if os.path.exists(d):
            target_dir = d
            break
            
    pkg_src_dir = os.path.join(target_dir, f'scan_{timestamp}')

    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', pkg_src_dir,
            '/scan',
            '/odom',
            '/ekf_odom',
            '/icp_pose_raw',
            '/scan_to_map_icp/confidence',
            '/map_scan',
            '/tf',
            '/tf_static'
        ],
        output='screen'
    )

    return LaunchDescription([
        record_process
    ])
