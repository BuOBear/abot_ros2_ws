from glob import glob
from setuptools import setup
setup(name='jy_real_robot', version='0.1.0', packages=['jy_real_robot'],
 data_files=[('share/ament_index/resource_index/packages',['resource/jy_real_robot']),
 ('share/jy_real_robot',['package.xml']),
 ('share/jy_real_robot/config',glob('config/*.yaml')),
 ('share/jy_real_robot/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'], zip_safe=True,
 maintainer='Project maintainer', maintainer_email='maintainer@example.com',
 description='Physical mobile manipulator integration', license='Apache-2.0',
 tests_require=['pytest'], entry_points={'console_scripts':[
 'preflight=jy_real_robot.preflight:main',
 'yolo11_detector=jy_real_robot.detector:main',
 'servo_arm=jy_real_robot.arm_node:main',
 'table_target=jy_real_robot.target_node:main']})
