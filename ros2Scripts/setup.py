from setuptools import setup

package_name = 'px_bridge_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='University of Edinburgh Robotics Project',
    maintainer_email='s2157069@ed.ac.uk',
    author='University of Edinburgh Robotics Project',
    author_email='s2157069@ed.ac.uk',
    description=(
        'Educational ROS 2 bridge package for the PincherX simulator. '
        'Streams Dynamixel servo position data from an RB-150 controller '
        'over serial and publishes it to control the simulated PincherX arm.'
    ),
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_bridge = px_bridge_pkg.teleop_bridge:main',
            # runs base_control.py’s main() when launched
            'base_control = px_bridge_pkg.base_control:main',
        ],
    },
)