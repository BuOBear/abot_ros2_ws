from setuptools import setup


package_name = 'abot_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abot maintainers',
    maintainer_email='maintainer@example.invalid',
    description='Offline speech recognition and opt-in local announcements for ABOT ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_transcriber = abot_voice.voice_transcriber:main',
            'voice_announcer = abot_voice.voice_announcer:main',
        ],
    },
)
