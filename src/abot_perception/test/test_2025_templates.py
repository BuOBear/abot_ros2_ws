import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.parameter import Parameter

from abot_perception.template_detector import TemplateBank, TemplateDetector


def installed_template_paths():
    package_share = Path(get_package_share_directory('abot_perception'))
    return package_share / 'templates', package_share / 'templates' / 'shoot_object2025'


def test_shoot_object2025_installed_assets_match_manifest():
    _, template_dir = installed_template_paths()
    manifest = json.loads((template_dir / 'MANIFEST.json').read_text(encoding='utf-8'))

    assert manifest['source_directory'] == 'src/abot_find/shoot_object2025'
    assert manifest['selection_rule'].startswith('all top-level PNG files are preserved;')
    assert manifest['asset_count'] == 86
    assert manifest['numeric_template_count'] == 77
    assert manifest['auxiliary_asset_count'] == 9

    entries = manifest['assets']
    assert len(entries) == manifest['asset_count']
    numeric_entries = [entry for entry in entries
                       if entry['asset_type'] == 'numeric_template']
    auxiliary_entries = [entry for entry in entries if entry['asset_type'] == 'auxiliary']
    assert len(numeric_entries) == manifest['numeric_template_count']
    assert len(auxiliary_entries) == manifest['auxiliary_asset_count']
    assert [int(entry['id']) for entry in numeric_entries] == sorted(
        int(entry['id']) for entry in numeric_entries)
    assert {entry['installed_path'] for entry in numeric_entries} == {
        path.name for path in template_dir.glob('*.png')
    }
    auxiliary_dir = template_dir / 'auxiliary'
    assert {entry['installed_path'] for entry in auxiliary_entries} == {
        path.relative_to(template_dir).as_posix()
        for path in auxiliary_dir.iterdir()
        if path.is_file() and path.name.lower().endswith('.png')
    }
    assert '.png' in {entry['filename'] for entry in auxiliary_entries}
    assert {f'MarkerData_{index}.png' for index in range(1, 9)} == {
        entry['filename'] for entry in auxiliary_entries
    } - {'.png'}

    for entry in entries:
        if entry['asset_type'] == 'numeric_template':
            assert entry['filename'] == f"{entry['id']}.png"
        asset_path = template_dir / entry['installed_path']
        data = asset_path.read_bytes()
        assert len(data) == entry['size_bytes']
        assert hashlib.sha256(data).hexdigest() == entry['sha256']
        assert cv2.imread(str(asset_path), cv2.IMREAD_UNCHANGED) is not None


def test_shoot_object2025_selection_uses_its_directory_and_keeps_ids_scoped():
    legacy_dir, template_dir = installed_template_paths()
    rclpy.init()
    node = None
    try:
        node = TemplateDetector(parameter_overrides=[
            Parameter('template_dir', value=str(template_dir)),
            Parameter('template_id', value='1'),
        ])
        assert node.template_ids == ('1',)
        assert [matcher.template_id for matcher in node.template_bank.matchers] == ['1']

        template = cv2.imread(str(template_dir / '1.png'), cv2.IMREAD_GRAYSCALE)
        image = np.full((480, 640), 150, dtype=np.uint8)
        height, width = template.shape
        image[100:100 + height, 280:280 + width] = template
        image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        message = CvBridge().cv2_to_imgmsg(image_bgr, encoding='bgr8')
        message.header.frame_id = 'camera_optical_frame'
        message.header.stamp = node.get_clock().now().to_msg()
        published = []
        node.detections = type('Recorder', (), {
            'publish': lambda _, detection_array: published.append(detection_array)
        })()
        node.on_image(message)
        assert [detection.results[0].hypothesis.class_id
                for detection in published[-1].detections] == ['template:1']

        # ID 1 exists only in the selected 2025 directory, not the original set.
        with pytest.raises(ValueError, match='unavailable'):
            TemplateBank(legacy_dir, ('1',))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
