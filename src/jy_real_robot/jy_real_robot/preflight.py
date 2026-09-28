"""Read-only deployment checks. Never opens serial/video devices."""
import argparse
import importlib.util
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from .core import local_weights
from .servo import load_profile


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-profile', default='')
    parser.add_argument('--arm-port', default='')
    parser.add_argument('--weights', default='')
    parser.add_argument('--camera-device', default='')
    options = parser.parse_args(args)
    checks = []

    def check(name, function):
        try:
            detail = function()
            checks.append({'check': name, 'ok': True, 'detail': str(detail)})
        except Exception as exc:
            checks.append({'check': name, 'ok': False, 'detail': str(exc)})

    for package in ('abot_description', 'abot_hardware', 'abot_bringup', 'abot_navigation',
                    'jy_real_interfaces', 'jy_real_robot'):
        check(package, lambda p=package: get_package_share_directory(p))
    if options.arm_profile:
        check('arm calibration', lambda: list(load_profile(options.arm_profile)))
    if options.weights:
        check('local detection weights', lambda: local_weights(options.weights))
        def ultralytics():
            if importlib.util.find_spec('ultralytics') is None:
                raise RuntimeError('ultralytics not installed in this Python environment')
            return 'installed; model compatibility still requires inference test'
        check('ultralytics', ultralytics)
    for name, filename in [('arm device', options.arm_port), ('camera device', options.camera_device)]:
        if filename:
            def device(path=filename):
                if not Path(path).is_char_device():
                    raise ValueError(f'character device missing: {path}')
                return f'{path} exists; not opened or functionally tested'
            check(name, device)
    print(json.dumps({'checks': checks,
                      'scope': 'Only requested components checked; not a motion/grasp readiness certificate'},
                     ensure_ascii=False, indent=2))
    return 0 if all(row['ok'] for row in checks) else 1
