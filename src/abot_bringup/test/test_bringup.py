"""Check composition guards before starting any physical process."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def module():
    path = Path(__file__).parents[1] / 'launch' / 'bringup.launch.py'
    spec = importlib.util.spec_from_file_location('bringup', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


@pytest.mark.parametrize('settings,reason', [
    ({'enable_hardware': 'true', 'use_sim_time': 'true'}, 'Physical base'),
    ({'enable_lidar': 'true', 'use_sim_time': 'true'}, 'Physical sensors'),
    ({'enable_camera': 'true', 'use_sim_time': 'true'}, 'Physical sensors'),
    ({'mode': 'mapping'}, 'state_estimation'),
    ({'mode': 'localization', 'enable_state_estimation': 'true'}, 'enable_velocity'),
])
def test_invalid_compositions_fail_before_start(settings, reason):
    context = LaunchContext()
    context.launch_configurations.update({
        'enable_hardware': 'false', 'enable_lidar': 'false', 'enable_camera': 'false',
        'use_sim_time': 'false', 'mode': 'disabled',
        'enable_state_estimation': 'false', 'enable_velocity': 'false', **settings})
    with pytest.raises(RuntimeError, match=reason):
        module()._compose(context)


def test_lidar_profile_matches_reported_x3_pro_and_uses_separate_port():
    config = Path(__file__).parents[1] / 'config'
    lidar = yaml.safe_load((config / 'lidar.yaml').read_text())
    hardware = yaml.safe_load((config / 'hardware.yaml').read_text())
    params = lidar['ydlidar_ros2_driver_node']['ros__parameters']
    base_params = hardware['abot_hardware']['ros__parameters']
    assert params['port'] != base_params['port']
    assert params['frame_id'] == 'laser_link'
    assert params['baudrate'] == 115200
    assert params['lidar_type'] == 1
    assert params['device_type'] == 0
    assert params['isSingleChannel'] is True
    assert params['sample_rate'] == 3
    assert params['range_min'] == 0.1
    assert params['range_max'] == 8.0


def test_user_selected_legacy_camera_calibration_is_installed():
    source_config = Path(__file__).parents[1] / 'config'
    camera = yaml.safe_load((source_config / 'camera.yaml').read_text())[
        'usb_cam']['ros__parameters']
    calibration = yaml.safe_load(
        (source_config / 'camera_calibration.yaml').read_text())
    assert (camera['image_width'], camera['image_height']) == (640, 480)
    assert (calibration['image_width'], calibration['image_height']) == (640, 480)
    assert camera['camera_name'] == calibration['camera_name'] == 'usb_cam'

    sensors_path = Path(__file__).parents[1] / 'launch' / 'sensors.launch.py'
    spec = importlib.util.spec_from_file_location('sensors', sensors_path)
    sensors = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sensors)
    context = LaunchContext()
    for action in sensors.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    url = LaunchConfiguration('camera_info_url').perform(context)
    assert url.startswith('file://')
    installed_calibration = Path(url.removeprefix('file://'))
    assert installed_calibration.is_file()
    assert yaml.safe_load(installed_calibration.read_text()) == calibration
