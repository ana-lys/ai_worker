from setuptools import find_packages, setup

package_name = 'ffw_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/record_map.launch.py']),
    ],
    install_requires=['setuptools', 'rosbag2_py', 'open3d', 'opencv-python', 'scikit-learn', 'pyyaml', 'numpy', 'tf-transformations'],
    zip_safe=True,
    maintainer='lys',
    maintainer_email='shatapdude@gmail.com',
    description='Offline Mapping tools using ICP and RANSAC',
    license='MIT',
    scripts=['scripts/generate_map.py', 'scripts/convert_bag.py'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
