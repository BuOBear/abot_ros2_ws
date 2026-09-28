import importlib.util
from pathlib import Path

import pytest
import rclpy
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from rclpy.parameter import Parameter

from abot_vlm.vlm_action import VlmAction


def test_launch_camera_default_matches_node():
    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'vlm.launch.py'
    spec = importlib.util.spec_from_file_location('abot_vlm_launch', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    image_argument = next(
        item for item in description.entities
        if isinstance(item, DeclareLaunchArgument) and item.name == 'image_topic')
    actual = ''.join(
        LaunchContext().perform_substitution(value)
        for value in image_argument.default_value)
    assert actual == '/camera/image_raw'


@pytest.mark.parametrize('name,value', [
    (name, value)
    for name in (
        'default_max_frame_age_sec', 'default_timeout_sec',
        'maximum_frame_age_sec', 'max_timeout_sec')
    for value in (-1.0, 0.0, float('nan'), float('inf'), -float('inf'))
] + [
    ('maximum_frame_age_sec', 61.0),
    ('max_timeout_sec', 121.0),
    ('default_max_frame_age_sec', 6.0),
    ('default_timeout_sec', 61.0),
])
def test_invalid_limit_parameter_rejected_at_construction(name, value):
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            VlmAction(parameter_overrides=[Parameter(name, value=value)])
    finally:
        rclpy.shutdown()


def test_valid_maximum_limit_parameters():
    rclpy.init()
    try:
        node = VlmAction(parameter_overrides=[
            Parameter('default_max_frame_age_sec', value=60.0),
            Parameter('maximum_frame_age_sec', value=60.0),
            Parameter('default_timeout_sec', value=120.0),
            Parameter('max_timeout_sec', value=120.0),
        ])
        assert node._maximum_age == 60.0
        assert node._maximum_timeout == 120.0
        node.destroy_node()
    finally:
        rclpy.shutdown()
