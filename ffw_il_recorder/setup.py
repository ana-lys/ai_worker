from setuptools import find_packages, setup

package_name = 'ffw_il_recorder'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/il_recorder_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Imitation-learning episode recorder (OAK-D head + right RGB, joint state, EE poses).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'il_episode_recorder = ffw_il_recorder.recorder:main',
        ],
    },
)
