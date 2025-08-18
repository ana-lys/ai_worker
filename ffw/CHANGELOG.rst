^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ffw
^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.10 (2025-07-22)
-------------------
* Added start/pause feature for ffw_joint_trajectory_command_broadcaster
* Merged two joint_trajectory_command_broadcaster into one for ffw_lg2_leader
* Added ffw_robot_manager package in meta package
* Added URDF for ZED camera
* Modified MoveIt files
* Updated Dockerfile
* Contributors: Wonho Yun, Woojin Wie

1.1.9 (2025-07-18)
------------------
* Added robot manager ros2 controller
* Added Swerve Steer initialization feature
* Fixed Gazebo simulation issue
* Added error code reading feature for Dynamixel
* Contributors: Woojin Wie

1.1.8 (2025-07-14)
------------------
* Modified joystick controller to enable lift control while in swerve mode
* Refactored joystick controller
* Contributors: Wonho Yun

1.1.7 (2025-07-11)
------------------
* Added slow start feature for joint_trajectory_command_broadcaster
* Contributors: Woojin Wie

1.1.6 (2025-07-07)
------------------
* Updated ros2_control xacro file for ffw_bg2_rev4_follower
* Fix header file name for tf2
* Contributors: Woojin Wie

1.1.5 (2025-06-30)
------------------
* Updated ros2_control xacro file for ffw_bg2_rev2_follower
* Contributors: Woojin Wie

1.1.4 (2025-06-27)
------------------
* Added ROS_DOMAIN_ID to the Dockerfile
* Contributors: Woojin Wie

1.1.3 (2025-06-26)
------------------
* Modified jog scale for ffw_lg2_leader
* Added dependencies to the package.xml file for image_transport_plugins
* Contributors: Woojin Wie

1.1.2 (2025-06-26)
------------------
* Added dependencies to the package.xml file
* Contributors: Woojin Wie

1.1.1 (2025-06-26)
------------------
* Reordered pip install order in Dockerfile to fix the numpy version issue
* Added Current Limit parameter to the ros2_control xacro file for ffw_sg2_rev1
* Contributors: Woojin Wie

1.1.0 (2025-06-16)
------------------
* Add installation of some ROS 2 packages for physical AI tools in Dockerfile
* Add an alias command in Dockerfile for running the physical AI server
* Support ffw_sg2_rev1 Model
* Add swerve drive controller package for ffw_sg2_rev1
* Modify joystick controller to support swerve mode
* Contributors: Kiwoong Park, Woojin Wie, Geonhee Lee, Wonho Yun

1.0.9 (2025-06-09)
------------------
* Updated urdf files for ffw_bg2_rev4
* Modified Gazebo launch file
* Contributors: Woojin Wie, Wonho Yun

1.0.8 (2025-06-02)
------------------
* Updated Model files for ffw_bg2_rev4
* Contributors: Woojin Wie

1.0.7 (2025-05-30)
------------------
* Updated Model files for ffw_bg2_rev4
* Contributors: Woojin Wie

1.0.6 (2025-05-28)
------------------
* Modified Docker volume mapping
* Created RealSense and ZED launch files
* Adjusted joint names
* Improved file structure
* Removed deprecated files
* Contributors: Woojin Wie

1.0.5 (2025-05-09)
------------------
* Fixed Dockerfile
* Updated Camera URDF
* Contributors: Woojin Wie

1.0.4 (2025-05-08)
------------------
* Fixed Dockerfile
* Updated ros2 control xacro file to support async
* Contributors: Woojin Wie

1.0.3 (2025-04-28)
------------------
* Added support for Joystick controller
* Added ffw_spring_actuator_controller
* Contributors: Woojin Wie

1.0.2 (2025-04-16)
------------------
* Added support for ROBOTIS RH Gripper
* Added differentiation between slow and fast versions
* Updated codebase to comply with flake8 linting standards
* Contributors: Wonho Yun

1.0.1 (2025-04-07)
------------------
* Modified the profile velocity parameters for enhanced arm and hand teleoperation performance
* Modified the README file to reflect usage instructions for this package
* Removed unused files and redundant comments to streamline the codebase
* Contributors: Wonho Yun, Pyo

1.0.0 (2025-04-06)
------------------
* Added the initial version of the FFW ROS package
* Added arm and hand teleoperation support for FFW
* Added integrated controller compatibility for Inspire Robot Hand
* Contributors: Sungho Woo, Woojin Wie, Wonho Yun, Pyo

0.1.0 (2025-03-27)
------------------
* Added bringup scripts for system initialization
* Added robot description files for visualization and planning
* Added base controller functionalities
* Added MoveIt for motion planning support
* Contributors: Sungho Woo, Woojin Wie
