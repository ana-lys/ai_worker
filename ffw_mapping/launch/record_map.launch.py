import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # We want to save the rosbag inside the source directory of the package
    # so it persists across builds and is easy to find.
    # Note: get_package_share_directory points to the install space. 
    # To save in the src space, we hardcode the relative path from workspace root, 
    # or just save to a known location. A safe bet is saving it in the current working directory,
    # but the user requested "folder scan in the package".
    
    # We will use an absolute path assuming the standard workspace setup
    pkg_src_dir = os.path.expanduser('~/robotis_ws/src/ai_worker/ffw_mapping/scan')
    
    # Create the directory if it doesn't exist
    os.makedirs(pkg_src_dir, exist_ok=True)

    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', pkg_src_dir, '/scan', '/ffw_laser_odom'],
        output='screen'
    )

    return LaunchDescription([
        record_process
    ])
