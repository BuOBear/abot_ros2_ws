from types import SimpleNamespace
import numpy as np
import pytest
from jy_real_robot.core import (fresh, LatestFrame, local_weights, detections_from_result,
    calibrated_ray, rotation_from_quaternion, intersect_plane)
from jy_real_robot.detector import to_message
from jy_real_robot.target_node import TableTarget, matching_camera_info, matching_image_size
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray


def test_temporal_contract_and_bounded_queue():
    assert fresh(1_000_000_000, 1_100_000_000, .2)
    assert not fresh(0, 10, .2)
    assert not fresh(1_000_000_000, 2_000_000_000, .2)
    assert not fresh(2_000_000_000, 1_000_000_000, .2)
    queue = LatestFrame()
    queue.put('old')
    queue.put('new')
    assert queue.take() == 'new'
    assert queue.take() is None


def test_missing_weights_do_not_download(tmp_path):
    with pytest.raises(ValueError):
        local_weights(str(tmp_path / 'yolo11n.pt'))


def test_real_classes_clipping_and_standard_message():
    class Boxes:
        xyxy = [[-10, 2, 40, 22], [2, 2, 5, 5], [1, 2, float('nan'), 5]]
        conf = [.9, .1, .99]
        cls = [2, 0, 0]
        def cpu(self):
            return self
        def numpy(self):
            return self
    rows = detections_from_result(SimpleNamespace(boxes=Boxes(), names={2: 'bottle'}), 640, 480, .5)
    assert rows == [('bottle', .9, 0, 2, 40, 22)]
    header = Header(frame_id='wrist_camera_optical_frame')
    header.stamp.sec = 12
    message = to_message(header, rows)
    assert message.header == header
    assert message.detections[0].results[0].hypothesis.class_id == 'bottle'
    assert message.detections[0].bbox.center.position.x == 20


def test_negative_class_index_cannot_select_last_model_label():
    class Boxes:
        xyxy = [[2, 2, 20, 20]]
        conf = [.9]
        cls = [-1]
        def cpu(self):
            return self
        def numpy(self):
            return self
    result = SimpleNamespace(boxes=Boxes(), names=['object'])
    assert detections_from_result(result, 640, 480, .5) == []


def test_plane_geometry_and_reject_behind_camera():
    k = [500, 0, 320, 0, 500, 240, 0, 0, 1]
    ray = calibrated_ray(320, 240, k, [0.] * 5, 'plumb_bob')
    rotation = rotation_from_quaternion([1, 0, 0, 0])
    assert np.allclose(intersect_plane(ray, [0, 0, 1], rotation, [0, 0, 1], 0), [0, 0, 0])
    with pytest.raises(ValueError):
        intersect_plane(ray, [0, 0, 1], np.eye(3), [0, 0, 1], 0)
    with pytest.raises(ValueError):
        intersect_plane([1, 0, 0], [0, 0, 1], np.eye(3), [0, 0, 1], 0)
    with pytest.raises(ValueError):
        calibrated_ray(320, 240, [0.] * 9, [], 'plumb_bob')


@pytest.mark.parametrize('matrix,distortion,model', [
    ([500, 20, 320, 0, 500, 240, 0, 0, 1], [0.] * 5, 'plumb_bob'),
    ([500, 0, 320, 0, 500, 240, 0, 0, 2], [0.] * 5, 'plumb_bob'),
    ([500, 0, 320, 0, 500, 240, 0, 0, 1], [], 'plumb_bob'),
    ([500, 0, 320, 0, 500, 240, 0, 0, 1], [0.] * 5, 'rational_polynomial'),
    ([500, 0, 320, 0, 500, 240, 0, 0, 1], [0.] * 3, 'equidistant'),
])
def test_unsupported_camera_calibration_is_rejected(matrix, distortion, model):
    with pytest.raises(ValueError):
        calibrated_ray(320, 240, matrix, distortion, model)


def test_camera_info_matches_capture_after_slow_inference():
    infos = []
    for frame in range(10):
        info = CameraInfo()
        info.header.frame_id = 'wrist_camera_optical_frame'
        info.header.stamp.sec = 12
        info.header.stamp.nanosec = frame * 33_000_000
        infos.append(info)
    detection_header = Header(frame_id='wrist_camera_optical_frame')
    detection_header.stamp.sec = 12
    detection_header.stamp.nanosec = 3 * 33_000_000
    assert matching_camera_info(detection_header, infos) is infos[3]
    detection_header.frame_id = 'other_camera'
    assert matching_camera_info(detection_header, infos) is None
    detection_header.frame_id = 'wrist_camera_optical_frame'
    detection_header.stamp.nanosec += 1
    assert matching_camera_info(detection_header, infos) is None


def test_source_image_resolution_must_match_calibration():
    header = Header(frame_id='wrist_camera_optical_frame')
    header.stamp.sec = 12
    images = [(capture_ns, 'wrist_camera_optical_frame', width, height)
              for capture_ns, width, height in [(12_000_000_000, 640, 480),
                                                 (12_033_000_000, 1280, 720)]]
    assert matching_image_size(header, images) == (640, 480)
    header.stamp.nanosec = 33_000_000
    assert matching_image_size(header, images) == (1280, 720)
    header.frame_id = 'other_camera'
    assert matching_image_size(header, images) is None

    header.frame_id = 'wrist_camera_optical_frame'
    header.stamp.nanosec = 0
    info = CameraInfo(header=header, width=640, height=480)
    fake_node = SimpleNamespace(infos=[info], images=[(12_000_000_000,
                      'wrist_camera_optical_frame', 1280, 720)])
    # The unbound callback reaches no clock, TF lookup or publisher on mismatch.
    TableTarget.detect(fake_node, Detection2DArray(header=header))
