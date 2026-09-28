from glob import glob
from setuptools import setup


package_name = 'abot_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abot maintainers',
    maintainer_email='maintainer@example.invalid',
    description='ROS 2 guarded visual and lidar following through the tracking velocity channel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'vision_follower = abot_tracking.vision_follower:main',
        'lidar_follower = abot_tracking.lidar_follower:main',
        'roi_lk_tracker = abot_tracking.roi_lk_tracker:main',
    ]},
)
