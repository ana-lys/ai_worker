from setuptools import find_packages, setup

package_name = 'ffw_zmqinterface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ana-lys',
    maintainer_email='shatapdude@gmail.com',
    description='Robot-side ZMQ HIL link: RobotState/EEState (PUB) out, ControlCmd (SUB) in; gateway bridges to the spacemouse teleop stack.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = ffw_zmqinterface.robot_node:main',
            'gateway_node = ffw_zmqinterface.gateway_node:main',
        ],
    },
)
