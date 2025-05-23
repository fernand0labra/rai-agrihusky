from setuptools import setup

package_name = 'agrihusky'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'fake_gps_pub',
        'fake_odometry_pub',
        'waypoint_pub',
        'probe_interface',
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
            'sim_visualize = agrihusky.visualizer:main',
            'sim_fake_gps  = agrihusky.fake_gps_pub:main'
            'sim_fake_odom = agrihusky.fake_odometry_pub:main',
            'probe         = agrihusky.probe_interface:main',
            'waypoint      = agrihusky.waypoint_pub:main',
            'control       = agrihusky.controller:main',
        ],
    },
)