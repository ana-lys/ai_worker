import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node

def get_joystick_id():
    """Shells out to joy_enumerate_devices to find the best RadioLink ID."""
    try:
        # Run the enumeration tool and capture output
        result = subprocess.check_output(['ros2', 'run', 'joy', 'joy_enumerate_devices'], 
                                       text=True, stderr=subprocess.DEVNULL)
        
        # Priority 1: R16F
        if "RadioLink R16F" in result:
            for line in result.splitlines():
                if "RadioLink R16F" in line:
                    return int(line.split(':')[0].strip())
        
        # Priority 2: T16D
        if "Radiolink T16D" in result:
            for line in result.splitlines():
                if "Radiolink T16D" in line:
                    return int(line.split(':')[0].strip())
                    
    except Exception:
        pass
    
    return 0  # Default fallback
def generate_launch_description():
    target_id = get_joystick_id()
    
    # Calculate the exact bias: (1063 - 1024) / 1024 approx 0.038
    # Since your Joy node reported -0.0381 for 1063, our offset is -0.0381
    radio_bias = -0.0381 

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': target_id,
                'deadzone': 0.0,
                'autorepeat_rate': 50.0,
            }]
        ),
        Node(
            package='ffw_radio',
            executable='radio_dashboard',
            name='radio_dashboard',
            output='screen',
            parameters=[{
                'center_offset': radio_bias ,
                'display_mode': 2 ,
            }]
        )
    ])