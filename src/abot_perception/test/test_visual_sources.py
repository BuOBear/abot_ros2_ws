from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter

from abot_perception.person_detector import PersonDetector, detect_people
from abot_perception.tag_detector import TagDetector, detect_tags, tag_dictionary
from abot_perception.template_detector import (
    MAX_TEMPLATE_IDS,
    TemplateBank,
    TemplateDetector,
    TemplateMatcher,
    resolve_template_ids,
)


def frame_message(image, node):
    message = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
    message.header.frame_id = 'camera_optical_frame'
    message.header.stamp = node.get_clock().now().to_msg()
    return message


def marker_frame(dictionary_name='DICT_APRILTAG_36h11'):
    dictionary = tag_dictionary(dictionary_name)
    marker = cv2.aruco.drawMarker(dictionary, 7, 160)
    image = np.full((300, 420, 3), 255, dtype=np.uint8)
    image[70:230, 190:350] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return image


def template_frame():
    path = Path(__file__).parents[1] / 'templates' / '10.png'
    template = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    assert template is not None
    image = np.full((480, 640), 150, dtype=np.uint8)
    height, width = template.shape
    image[100:100 + height, 280:280 + width] = template
    return template, cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def template_id_frame(template_id):
    path = Path(__file__).parents[1] / 'templates' / f'{template_id}.png'
    template = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    assert template is not None
    image = np.full((480, 640), 150, dtype=np.uint8)
    height, width = template.shape
    image[100:100 + height, 280:280 + width] = template
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def template_composite_frame():
    template_dir = Path(__file__).parents[1] / 'templates'
    image = np.full((480, 640), 150, dtype=np.uint8)
    for template_id, x in (('10', 20), ('20', 370)):
        template = cv2.imread(str(template_dir / f'{template_id}.png'), cv2.IMREAD_GRAYSCALE)
        assert template is not None
        height, width = template.shape
        image[100:100 + height, x:x + width] = template
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def test_tag_dictionary_detection_and_rejection():
    image = marker_frame()
    found = detect_tags(image, 'DICT_APRILTAG_36h11',
                        tag_dictionary('DICT_APRILTAG_36h11'))
    assert len(found) == 1
    assert found[0].class_id == 'tag:apriltag_36h11:7'
    assert found[0].center_x > 250
    assert detect_tags(np.full_like(image, 255), 'DICT_APRILTAG_36h11',
                       tag_dictionary('DICT_APRILTAG_36h11')) == []
    with pytest.raises(ValueError):
        tag_dictionary('DICT_UNKNOWN')


def test_tag_non_default_dictionary_detection():
    dictionary_name = 'DICT_5X5_250'
    found = detect_tags(marker_frame(dictionary_name), dictionary_name,
                        tag_dictionary(dictionary_name))
    assert [item.class_id for item in found] == ['tag:5x5_250:7']


def test_template_homography_and_blank_rejection():
    template, image = template_frame()
    matcher = TemplateMatcher(template, '10')
    found = matcher.detect(image)
    assert len(found) == 1
    assert found[0].class_id == 'template:10'
    assert 350 < found[0].center_x < 480
    assert matcher.detect(np.full_like(image, 150)) == []


def test_template_accepts_same_object_variant_and_rejects_other_asset():
    template10 = cv2.imread(
        str(Path(__file__).parents[1] / 'templates' / '10.png'), cv2.IMREAD_GRAYSCALE)
    matcher = TemplateMatcher(template10, '10')
    variant11 = matcher.detect(template_id_frame('11'))
    different20 = matcher.detect(template_id_frame('20'))
    assert len(variant11) == 1
    assert variant11[0].class_id == 'template:10'
    assert different20 == []


class CountingSift:
    def __init__(self):
        self.sift = cv2.SIFT_create(nfeatures=1200)
        self.calls = 0

    def detectAndCompute(self, image, mask):
        self.calls += 1
        return self.sift.detectAndCompute(image, mask)


def test_template_bank_detects_two_assets_with_one_frame_sift_pass():
    template_dir = Path(__file__).parents[1] / 'templates'
    feature = CountingSift()
    bank = TemplateBank(template_dir, ('10', '20'), feature=feature)
    assert feature.calls == 2  # Each installed template is described once at startup.

    feature.calls = 0
    found10 = bank.detect(template_id_frame('10'))
    assert [item.class_id for item in found10] == ['template:10']
    assert feature.calls == 1

    feature.calls = 0
    found20 = bank.detect(template_id_frame('20'))
    assert [item.class_id for item in found20] == ['template:20']
    assert feature.calls == 1

    feature.calls = 0
    composite = bank.detect(template_composite_frame())
    assert [item.class_id for item in composite] == ['template:10', 'template:20']
    assert feature.calls == 1


def test_template_ids_precedence_validation_and_missing_assets():
    assert MAX_TEMPLATE_IDS == 8
    assert resolve_template_ids('', '10') == ('10',)
    assert resolve_template_ids('10, 20', '999') == ('10', '20')
    for value in ('10,abc', '10,,20', '10,   ,20', '10,10', '../10', '010'):
        with pytest.raises(ValueError):
            resolve_template_ids(value, '10')
    with pytest.raises(ValueError, match='at most'):
        resolve_template_ids(','.join(
            ('10', '11', '12', '13', '14', '15', '17', '18', '20')), '10')
    with pytest.raises(ValueError, match='unavailable'):
        TemplateBank(Path(__file__).parents[1] / 'templates', ('30',))


def test_multi_template_ros_source_preserves_stamp_and_clears_on_loss(monkeypatch):
    monkeypatch.setattr('abot_perception.template_detector.get_package_share_directory',
                        lambda _: str(Path(__file__).parents[1]))
    rclpy.init()
    node = None
    try:
        node = TemplateDetector(parameter_overrides=[
            Parameter('template_ids', value='10,20'),
            Parameter('template_id', value='999'),
        ])
        assert node.template_ids == ('10', '20')
        published = []
        node.detections = type('Recorder', (),
                               {'publish': lambda _, msg: published.append(msg)})()

        for template_id in ('10', '20'):
            image = template_id_frame(template_id)
            message = frame_message(image, node)
            node.last_processed = None
            node.on_image(message)
            assert published[-1].header == message.header
            assert published[-1].detections[0].header == message.header
            assert [d.results[0].hypothesis.class_id
                    for d in published[-1].detections] == [f'template:{template_id}']

        composite = frame_message(template_composite_frame(), node)
        node.last_processed = None
        node.on_image(composite)
        assert published[-1].header == composite.header
        assert [d.results[0].hypothesis.class_id
                for d in published[-1].detections] == ['template:10', 'template:20']
        assert all(d.header == composite.header for d in published[-1].detections)

        blank = frame_message(np.full((480, 640, 3), 150, dtype=np.uint8), node)
        node.last_processed = None
        node.on_image(blank)
        assert published[-1].header == blank.header
        assert published[-1].detections == []
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_all_74_template_assets_initialize_with_default_thresholds():
    paths = sorted((Path(__file__).parents[1] / 'templates').glob('*.png'))
    assert len(paths) == 74
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        assert image is not None, path.name
        TemplateMatcher(image, path.stem)


def test_template_configuration_rejects_malformed_values():
    template = np.zeros((64, 64), dtype=np.uint8)
    for template_id, min_matches, min_inliers in [
            ('../10', 12, 8), ('10', 3, 2), ('10', 12, 13), ('10', 12.5, 8)]:
        with pytest.raises(ValueError):
            TemplateMatcher(template, template_id, min_matches, min_inliers)


class FakeHog:
    def __init__(self, found):
        self.found = found

    def detectMultiScale(self, *_args, **_kwargs):
        return self.found


def test_person_hog_filter_and_image_loss():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    boxes = detect_people(image, FakeHog(([(150, 40, 60, 140), (0, 0, 30, 30)],
                                          [0.8, 0.9])))
    assert len(boxes) == 1
    assert boxes[0].class_id == 'person'
    assert boxes[0].center_x == 180
    assert detect_people(image, FakeHog(([], []))) == []
    with pytest.raises(ValueError):
        detect_people(image, FakeHog(([], [])), min_weight=float('nan'))


@pytest.mark.parametrize('detector_type,image_factory,class_id', [
    (TagDetector, marker_frame, 'tag:apriltag_36h11:7'),
    (TemplateDetector, lambda: template_frame()[1], 'template:10'),
])
def test_ros_source_preserves_stamp_and_publishes_empty(
        detector_type, image_factory, class_id, monkeypatch):
    if detector_type is TemplateDetector:
        monkeypatch.setattr('abot_perception.template_detector.get_package_share_directory',
                            lambda _: str(Path(__file__).parents[1]))
    rclpy.init()
    node = None
    try:
        node = detector_type()
        published = []
        node.detections = type('Recorder', (),
                               {'publish': lambda _, msg: published.append(msg)})()
        image = image_factory()
        message = frame_message(image, node)
        node.on_image(message)
        assert published[-1].header == message.header
        assert published[-1].detections[0].header == message.header
        assert published[-1].detections[0].results[0].hypothesis.class_id == class_id
        node.last_processed = None
        blank = frame_message(np.full_like(image, 150), node)
        node.on_image(blank)
        assert published[-1].header == blank.header
        assert published[-1].detections == []

        # A new malformed frame clears detections immediately and retains the
        # acquisition header instead of stamping the result with local time.
        node.last_processed = None
        node.on_image(frame_message(image_factory(), node))
        malformed = frame_message(image_factory(), node)
        malformed.step = 0
        node.on_image(malformed)
        assert published[-1].header == malformed.header
        assert published[-1].detections == []
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_ros_person_source_preserves_stamp_and_publishes_empty():
    rclpy.init()
    node = None
    try:
        node = PersonDetector()
        node.hog = FakeHog(([(150, 40, 60, 140)], [0.8]))
        published = []
        node.detections = type('Recorder', (),
                               {'publish': lambda _, msg: published.append(msg)})()
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        message = frame_message(image, node)
        node.on_image(message)
        assert published[-1].header == message.header
        assert published[-1].detections[0].results[0].hypothesis.class_id == 'person'
        node.hog = FakeHog(([], []))
        node.last_processed = None
        node.on_image(message)
        assert published[-1].detections == []

        node.hog = FakeHog(([(150, 40, 60, 140)], [0.8]))
        node.last_processed = None
        node.on_image(message)
        malformed = frame_message(image, node)
        malformed.step = 0
        node.on_image(malformed)
        assert published[-1].header == malformed.header
        assert published[-1].detections == []
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
