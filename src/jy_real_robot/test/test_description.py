import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from jy_real_robot.description import base_description


def test_base_has_no_obsolete_camera_tf():
    filename = Path(get_package_share_directory('abot_description')) / 'urdf/abot.urdf.xacro'
    root = ET.fromstring(base_description(filename))
    names = {element.get('name') for element in root}
    assert {'base_link', 'base_footprint', 'laser_link', 'imu_link'} <= names
    assert not {'camera_link', 'camera_joint', 'camera_optical_frame'} & names
    for joint in root.findall('joint'):
        assert joint.find('parent').get('link') in names
        assert joint.find('child').get('link') in names
