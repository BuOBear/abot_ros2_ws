from glob import glob
from pathlib import Path
from setuptools import setup


package_name = 'abot_perception'
auxiliary_templates_dir = Path('templates/shoot_object2025/auxiliary')
auxiliary_template_assets = [
    str(path) for path in sorted(auxiliary_templates_dir.iterdir(),
                                 key=lambda item: item.name.lower())
    if path.is_file() and path.name.lower().endswith('.png')
]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/cascades', glob('cascades/*.xml')),
        ('share/' + package_name + '/templates', glob('templates/*.png') +
         ['templates/LICENSE.find-object']),
        ('share/' + package_name + '/templates/shoot_object2025',
         glob('templates/shoot_object2025/*.png') +
         ['templates/shoot_object2025/MANIFEST.json']),
        ('share/' + package_name + '/templates/shoot_object2025/auxiliary',
         auxiliary_template_assets),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abot maintainers',
    maintainer_email='maintainer@example.invalid',
    description='ROS 2 visual detections from camera images',
    license=('Apache-2.0 for package code; BSD-3-Clause for original template assets; '
             'shoot_object2025 source license is not declared separately; '
             'Intel License Agreement for Open Source Computer Vision Library '
             'for the Haar cascade asset'),
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'color_detector = abot_perception.color_detector:main',
        'tag_detector = abot_perception.tag_detector:main',
        'template_detector = abot_perception.template_detector:main',
        'person_detector = abot_perception.person_detector:main',
        'face_detector = abot_perception.face_detector:main',
        'fire_detector = abot_perception.fire_detector:main',
    ]},
)
