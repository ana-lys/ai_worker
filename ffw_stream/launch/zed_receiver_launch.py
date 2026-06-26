from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    base_port_arg = DeclareLaunchArgument(
        'base_port',
        default_value='9000',
        description='Base port to receive on (ZED streams to base_port + 100)'
    )
    
    # We can use a pure GStreamer pipeline to receive and display the ZED H264 stream
    # ZED streams to base_port + 100
    # Note: launch substitutions inside strings can be tricky, so we use a bash wrapper
    
    receiver_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'gst-launch-1.0 -v udpsrc port=$(($0 + 100)) '
            'caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264" ! '
            'rtph264depay ! decodebin ! videoconvert ! autovideosink sync=false',
            LaunchConfiguration('base_port')
        ],
        output='screen'
    )

    return LaunchDescription([
        base_port_arg,
        receiver_cmd
    ])
