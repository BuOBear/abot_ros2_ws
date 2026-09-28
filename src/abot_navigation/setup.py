from glob import glob
from setuptools import setup


package_name = 'abot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abot maintainers',
    maintainer_email='maintainer@example.invalid',
    description='Humble localization, omnidirectional Nav2 and controlled velocity chain',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_authority = abot_navigation.velocity_authority:main',
            'velocity_gate = abot_navigation.velocity_gate:main',
        ],
    },
)
