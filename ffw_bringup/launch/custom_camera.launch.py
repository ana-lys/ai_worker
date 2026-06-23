#!/usr/bin/env python3

import copy
import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Add realsense2_camera/launch to sys.path
realsense2_camera_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
sys.path.append(realsense2_camera_launch_dir)
import rs_launch

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def set_configurable_parameters(local_params):
    return {param['original_name']: LaunchConfiguration(param['name'])
            for param in local_params}

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def generate_launch_description():
    bringup_launch_dir = os.path.join(get_package_share_directory('ffw_bringup'), 'launch')

    # Read serial numbers
    serials_path = os.path.join(get_package_share_directory('ffw_bringup'), 'config', 'common', 'rs_serial.yaml')
    serials = yaml_to_dict(serials_path)
    serial1 = serials.get('camera1_serial')

    # ZED Camera (with performance depth and 720p as modified in yaml)
    camera_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'camera_zed.launch.py')),
        launch_arguments={'camera_model': 'zedm'}.items()
    )

    # Left D405 Only
    local_parameters = [
        {'name': 'camera_name1', 'default': 'camera_left', 'description': 'camera1 unique name'},
        {'name': 'camera_namespace1', 'default': 'camera_left', 'description': 'camera1 namespace'},
        {'name': 'serial_no1', 'default': serial1, 'description': 'choose device1 by serial number'},
        {'name': 'depth_module.depth_profile1', 'default': '480,270,30', 'description': 'depth stream profile for camera1'},
        {'name': 'depth_module.color_profile1', 'default': '424,240,30', 'description': 'Depth module color stream profile for d405 camera1'},
        {'name': 'colorizer.enable1', 'default': 'true', 'description': 'enable colorizer filter for camera1'},
    ]

    params1 = duplicate_params(rs_launch.configurable_parameters, '1')

    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        [
            camera_zed,
            TimerAction(
                period=5.0, 
                actions=[
                    OpaqueFunction(
                        function=rs_launch.launch_setup,
                        kwargs={
                            'params': set_configurable_parameters(params1),
                            'param_name_suffix': '1'
                        }
                    )
                ]
            )
        ]
    )
