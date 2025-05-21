from setuptools import setup

package_name = 'agrihusky'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'fake_gps_pub',
        'fake_odometry_pub',
        'fake_waypoint_pub',
        'visualizer',
        'controller',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fernando Labra Caso',
    maintainer_email='fernandolabracaso@gmail.com',
    description='A package for controlling the Husky robot with odometry and path planning in ROS 2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_gps = agrihusky.fake_gps_pub:main'
            'fake_odom = agrihusky.fake_odometry_pub:main',
            'fake_waypoint = agrihusky.fake_waypoint_pub:main',
            'visualize = agrihusky.visualizer:main',
            'control = agrihusky.controller:main',
        ],
    },
)